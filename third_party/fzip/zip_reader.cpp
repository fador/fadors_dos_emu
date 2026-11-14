// fzip — ZIP container reader implementation.
#include "zip_reader.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <span>
#include <stdexcept>
#include <thread>
#include <utility>

#include "codec.hpp"
#include "crc32.hpp"
#include "io.hpp"

namespace fzip {

namespace {

auto read_u16(const std::byte* p) -> std::uint16_t {
    return static_cast<std::uint16_t>(p[0]) |
           (static_cast<std::uint16_t>(p[1]) << 8);
}
auto read_u32(const std::byte* p) -> std::uint32_t {
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) |
           (static_cast<std::uint32_t>(p[3]) << 24);
}
auto read_u64(const std::byte* p) -> std::uint64_t {
    std::uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= (static_cast<std::uint64_t>(p[i]) << (8 * i));
    return v;
}

// Search backwards from `end` for the EOCD signature (PK\x05\x06).
auto find_eocd(const std::byte* data, std::size_t size) -> const std::byte* {
    // EOCD is at least 22 bytes; search from the end.
    if (size < 22) return nullptr;
    for (std::size_t i = size - 22; i > 0; --i) {
        if (read_u32(data + i) == 0x06054b50u) {
            return data + i;
        }
    }
    // Check offset 0.
    if (read_u32(data) == 0x06054b50u) return data;
    return nullptr;
}

constexpr std::uint16_t ZIP_METHOD_STORE = 0;
constexpr std::uint16_t ZIP_METHOD_SHRINK = 1;
constexpr std::uint16_t ZIP_METHOD_IMPLODE = 6;
constexpr std::uint16_t ZIP_METHOD_DEFLATE = 8;
constexpr std::uint16_t ZIP_METHOD_ZSTD = 93;

constexpr std::uint16_t ZIP_FLAG_ENCRYPTED = 0x0001u;
constexpr std::uint16_t IMPLODE_FLAG_8K_DICT = 0x0002u;
constexpr std::uint16_t IMPLODE_FLAG_3_TREES = 0x0004u;
constexpr std::uint32_t DOS_ATTR_DIR = 0x00000010u;

// Parse one central-directory entry at `p`. Returns the number of bytes
// consumed (46 + name_len + extra_len + comment_len), or 0 on error.
auto parse_cd_entry(const std::byte* p, const std::byte* end,
                    ZipReadEntry& out) -> std::size_t {
    if (p + 46 > end) { return 0; }
    if (read_u32(p) != 0x02014b50u) { return 0; }  // not a CD header

    // Version made by (2), version needed (2), flags (2), method (2),
    // time (2), date (2), crc (4), comp_size (4), uncomp_size (4),
    // name_len (2), extra_len (2), comment_len (2), disk_start (2),
    // internal_attr (2), external_attr (4), local_header_offset (4).
    out.flags = read_u16(p + 8);
    out.method = read_u16(p + 10);
    out.crc32 = read_u32(p + 16);
    out.compressed_size = read_u32(p + 20);
    out.uncompressed_size = read_u32(p + 24);
    auto name_len = read_u16(p + 28);
    auto extra_len = read_u16(p + 30);
    auto comment_len = read_u16(p + 32);
    auto external_attr = read_u32(p + 38);
    out.local_header_offset = read_u32(p + 42);

    if (p + 46 + name_len > end) { return 0; }
    out.name = std::string(reinterpret_cast<const char*>(p + 46), name_len);

    // Check for ZIP64 extra field (0x0001) to get 64-bit sizes/offset.
    const std::byte* extra = p + 46 + name_len;
    const std::byte* extra_end = extra + extra_len;
    if (extra_end > end) { extra_end = end; }
    while (extra + 4 <= extra_end) {
        auto eid = read_u16(extra);
        auto esz = read_u16(extra + 2);
        if (extra + 4 + esz > extra_end) { break; }
        if (eid == 0x0001u) {
            const std::byte* ed = extra + 4;
            int off = 0;
            // ZIP64 extra: uncompressed (8), compressed (8), offset (8),
            // disk (4) — but only the fields whose 32-bit counterpart
            // was the sentinel 0xFFFFFFFF are present.
            if (out.uncompressed_size == 0xFFFFFFFFu && off + 8 <= esz) {
                out.uncompressed_size = read_u64(ed + off); off += 8;
            }
            if (out.compressed_size == 0xFFFFFFFFu && off + 8 <= esz) {
                out.compressed_size = read_u64(ed + off); off += 8;
            }
            if (out.local_header_offset == 0xFFFFFFFFu && off + 8 <= esz) {
                out.local_header_offset = read_u64(ed + off); off += 8;
            }
        }
        extra += 4 + esz;
    }

    const bool ends_with_sep =
        !out.name.empty() && (out.name.back() == '/' || out.name.back() == '\\');
    const bool has_dos_dir_attr =
        (external_attr & DOS_ATTR_DIR) != 0 && out.uncompressed_size == 0;
    out.is_directory = ends_with_sep || has_dos_dir_attr;

    // Map method ID to CodecId.
    switch (out.method) {
        case ZIP_METHOD_STORE: out.codec = CodecId::Store; break;
        case ZIP_METHOD_DEFLATE: out.codec = CodecId::Deflate; break;
        case ZIP_METHOD_ZSTD: out.codec = CodecId::Zstd; break;
        default: out.codec = CodecId::Store; break;
    }

    return 46 + name_len + extra_len + comment_len;
}

}  // namespace

namespace {

auto parse_zip_entries(std::span<const std::byte> data)
    -> std::vector<ZipReadEntry> {
    const std::byte* base = data.data();
    const std::byte* end = base + data.size();

    const std::byte* eocd = find_eocd(base, data.size());
    if (!eocd) throw std::runtime_error("no EOCD found");

    // Parse EOCD: get CD offset and size.
    auto cd_size_32 = read_u32(eocd + 12);
    auto cd_offset_32 = read_u32(eocd + 16);
    auto total_entries = read_u16(eocd + 10);

    std::uint64_t cd_offset = cd_offset_32;
    std::uint64_t cd_size = cd_size_32;
    std::uint64_t entry_count = total_entries;

    // If sentinels, look for ZIP64 EOCD64 (PK\x06\x06) which precedes EOCD.
    if (cd_offset_32 == 0xFFFFFFFFu || cd_size_32 == 0xFFFFFFFFu ||
        total_entries == 0xFFFFu) {
        // EOCD64 is at eocd64_off from the locator (PK\x06\x07) which is
        // immediately before the EOCD.
        if (eocd - 20 >= base && read_u32(eocd - 20) == 0x07064b50u) {
            auto eocd64_off = read_u64(eocd - 20 + 8);
            const std::byte* e64 = base + eocd64_off;
            if (e64 + 56 <= end && read_u32(e64) == 0x06064b50u) {
                entry_count = read_u64(e64 + 32);
                cd_size = read_u64(e64 + 40);
                cd_offset = read_u64(e64 + 48);
            }
        }
    }

    if (cd_offset + cd_size > data.size()) {
        throw std::runtime_error("CD offset out of bounds");
    }

    std::vector<ZipReadEntry> entries;
    const std::byte* p = base + cd_offset;
    const std::byte* cd_end = p + cd_size;
    for (std::uint64_t i = 0; i < entry_count && p < cd_end; ++i) {
        ZipReadEntry e;
        auto consumed = parse_cd_entry(p, cd_end, e);
        if (consumed == 0) break;
        entries.push_back(std::move(e));
        p += consumed;
    }
    return entries;
}

}  // namespace

auto read_zip_entries(const std::string& archive_path)
    -> std::vector<ZipReadEntry> {
    auto data = io::read_file(archive_path);
    return parse_zip_entries(data);
}

namespace {

class LsbBitReader {
public:
    explicit LsbBitReader(std::span<const std::byte> data) : m_data(data) {}

    bool read_bits(int count, std::uint32_t& out) {
        if (!ensure(count)) { return false; }
        const std::uint64_t mask = (count == 32)
            ? 0xFFFFFFFFULL
            : ((1ULL << count) - 1ULL);
        out = static_cast<std::uint32_t>(m_bit_buf & mask);
        m_bit_buf >>= count;
        m_bit_cnt -= count;
        return true;
    }

private:
    bool ensure(int needed) {
        while (m_bit_cnt < needed && m_pos < m_data.size()) {
            const auto byte_val = std::to_integer<std::uint64_t>(m_data[m_pos++]);
            m_bit_buf |= (byte_val << m_bit_cnt);
            m_bit_cnt += 8;
        }
        return m_bit_cnt >= needed;
    }

    std::span<const std::byte> m_data;
    std::size_t m_pos = 0;
    std::uint64_t m_bit_buf = 0;
    int m_bit_cnt = 0;
};

constexpr std::size_t SHRINK_TABLE_SIZE = 8192;
constexpr int SHRINK_FIRST_FREE = 257;
constexpr int SHRINK_CONTROL_CODE = 256;
constexpr std::uint32_t SHRINK_INC_BITS = 1;
constexpr std::uint32_t SHRINK_PARTIAL_CLEAR = 2;
constexpr int SHRINK_INIT_BITS = 9;
constexpr int SHRINK_MAX_BITS = 13;

auto unshrink_decompress(std::span<const std::byte> comp_data,
                         std::uint64_t expected_size)
    -> std::vector<std::byte> {
    std::vector<std::byte> out;
    if (expected_size == 0) { return out; }
    out.reserve(static_cast<std::size_t>(expected_size));

    std::vector<int> parent(SHRINK_TABLE_SIZE, -1);
    std::vector<std::uint8_t> suffix(SHRINK_TABLE_SIZE, 0);
    for (int i = 0; i < SHRINK_CONTROL_CODE; ++i) {
        suffix[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(i);
    }

    auto partial_clear = [&]() {
        std::array<bool, SHRINK_TABLE_SIZE> has_child{};
        for (int i = SHRINK_FIRST_FREE; i < static_cast<int>(SHRINK_TABLE_SIZE); ++i) {
            const int p = parent[static_cast<std::size_t>(i)];
            if (p >= SHRINK_FIRST_FREE) {
                has_child[static_cast<std::size_t>(p)] = true;
            }
        }
        for (int i = SHRINK_FIRST_FREE; i < static_cast<int>(SHRINK_TABLE_SIZE); ++i) {
            if (!has_child[static_cast<std::size_t>(i)]) {
                parent[static_cast<std::size_t>(i)] = -1;
            }
        }
    };

    LsbBitReader br(comp_data);
    int code_size = SHRINK_INIT_BITS;
    int next_free = SHRINK_FIRST_FREE;

    std::uint32_t raw = 0;
    while (true) {
        if (!br.read_bits(code_size, raw)) { return out; }
        if (raw == SHRINK_CONTROL_CODE) {
            std::uint32_t ctrl = 0;
            if (!br.read_bits(code_size, ctrl)) {
                throw std::runtime_error("shrink truncated control code");
            }
            if (ctrl == SHRINK_INC_BITS && code_size < SHRINK_MAX_BITS) {
                ++code_size;
            } else if (ctrl == SHRINK_PARTIAL_CLEAR) {
                partial_clear();
                next_free = SHRINK_FIRST_FREE;
            }
            continue;
        }
        break;
    }

    if (raw >= static_cast<std::uint32_t>(SHRINK_CONTROL_CODE)) {
        throw std::runtime_error("shrink invalid initial code");
    }

    int oldcode = static_cast<int>(raw);
    std::uint8_t finchar = static_cast<std::uint8_t>(oldcode);
    out.push_back(static_cast<std::byte>(finchar));

    std::vector<std::uint8_t> stack;
    stack.reserve(256);

    while (out.size() < expected_size) {
        if (!br.read_bits(code_size, raw)) { break; }
        if (raw == SHRINK_CONTROL_CODE) {
            std::uint32_t ctrl = 0;
            if (!br.read_bits(code_size, ctrl)) {
                throw std::runtime_error("shrink truncated control code");
            }
            if (ctrl == SHRINK_INC_BITS) {
                if (code_size < SHRINK_MAX_BITS) { ++code_size; }
            } else if (ctrl == SHRINK_PARTIAL_CLEAR) {
                partial_clear();
                next_free = SHRINK_FIRST_FREE;
            } else {
                throw std::runtime_error("shrink unknown control subcode");
            }
            continue;
        }

        const int incode = static_cast<int>(raw);
        if (incode >= static_cast<int>(SHRINK_TABLE_SIZE)) {
            throw std::runtime_error("shrink code out of range");
        }

        stack.clear();
        int cur = incode;
        if (cur >= SHRINK_FIRST_FREE && parent[static_cast<std::size_t>(cur)] == -1) {
            stack.push_back(finchar);
            cur = oldcode;
        }

        while (cur >= SHRINK_FIRST_FREE) {
            if (parent[static_cast<std::size_t>(cur)] == -1 ||
                stack.size() >= SHRINK_TABLE_SIZE) {
                throw std::runtime_error("shrink corrupt dictionary chain");
            }
            stack.push_back(suffix[static_cast<std::size_t>(cur)]);
            cur = parent[static_cast<std::size_t>(cur)];
        }
        if (cur >= SHRINK_CONTROL_CODE) {
            throw std::runtime_error("shrink invalid root code");
        }
        finchar = static_cast<std::uint8_t>(cur);
        stack.push_back(finchar);

        for (auto it = stack.rbegin();
             it != stack.rend() && out.size() < expected_size; ++it) {
            out.push_back(static_cast<std::byte>(*it));
        }

        while (next_free < static_cast<int>(SHRINK_TABLE_SIZE) &&
               parent[static_cast<std::size_t>(next_free)] != -1) {
            ++next_free;
        }
        if (next_free < static_cast<int>(SHRINK_TABLE_SIZE)) {
            parent[static_cast<std::size_t>(next_free)] = oldcode;
            suffix[static_cast<std::size_t>(next_free)] = finchar;
            ++next_free;
        }
        oldcode = incode;
    }

    return out;
}

struct SfTrieNode {
    int child[2] = {-1, -1};
    int sym = -1;
};

struct SfTree {
    std::vector<SfTrieNode> nodes;

    int decode(LsbBitReader& br) const {
        int node = 0;
        while (nodes[static_cast<std::size_t>(node)].sym < 0) {
            std::uint32_t bit = 0;
            if (!br.read_bits(1, bit)) {
                throw std::runtime_error("explode unexpected end of bitstream");
            }
            node = nodes[static_cast<std::size_t>(node)].child[bit & 1u];
            if (node < 0) {
                throw std::runtime_error("explode invalid Shannon-Fano code");
            }
        }
        return nodes[static_cast<std::size_t>(node)].sym;
    }
};

auto bit_reverse_16(std::uint16_t val) -> std::uint16_t {
    std::uint16_t rev = 0;
    for (int i = 0; i < 16; ++i) {
        rev = static_cast<std::uint16_t>((rev << 1) | (val & 1u));
        val >>= 1;
    }
    return rev;
}

auto read_sf_tree(LsbBitReader& br, std::size_t num_syms) -> SfTree {
    std::uint32_t byte0 = 0;
    if (!br.read_bits(8, byte0)) {
        throw std::runtime_error("explode truncated tree header");
    }
    const std::size_t run_count = static_cast<std::size_t>(byte0) + 1;

    struct SymLen {
        int bitlen;
        int sym;
    };
    std::vector<SymLen> entries;
    entries.reserve(num_syms);

    int next_sym = 0;
    for (std::size_t r = 0; r < run_count; ++r) {
        std::uint32_t run_byte = 0;
        if (!br.read_bits(8, run_byte)) {
            throw std::runtime_error("explode truncated tree run");
        }
        const int count = static_cast<int>((run_byte >> 4) & 0x0Fu) + 1;
        const int bitlen = static_cast<int>(run_byte & 0x0Fu) + 1;
        for (int k = 0; k < count; ++k) {
            if (next_sym >= static_cast<int>(num_syms)) {
                throw std::runtime_error("explode tree symbol overflow");
            }
            entries.push_back({bitlen, next_sym++});
        }
    }
    if (entries.size() != num_syms) {
        throw std::runtime_error("explode tree symbol count mismatch");
    }

    std::sort(entries.begin(), entries.end(),
              [](const SymLen& a, const SymLen& b) {
                  if (a.bitlen != b.bitlen) { return a.bitlen < b.bitlen; }
                  return a.sym < b.sym;
              });

    SfTree tree;
    tree.nodes.push_back(SfTrieNode{});

    std::uint32_t code = 0;
    std::uint32_t code_inc = 0;
    int last_len = 0;
    for (int i = static_cast<int>(entries.size()) - 1; i >= 0; --i) {
        code += code_inc;
        const int blen = entries[static_cast<std::size_t>(i)].bitlen;
        if (blen != last_len) {
            last_len = blen;
            code_inc = 1u << (16 - blen);
        }
        const std::uint16_t rev = bit_reverse_16(static_cast<std::uint16_t>(code));
        int node = 0;
        for (int b = 0; b < blen; ++b) {
            const int bit = (rev >> b) & 1;
            if (tree.nodes[static_cast<std::size_t>(node)].child[bit] < 0) {
                const int next_idx = static_cast<int>(tree.nodes.size());
                tree.nodes[static_cast<std::size_t>(node)].child[bit] = next_idx;
                tree.nodes.push_back(SfTrieNode{});
            }
            node = tree.nodes[static_cast<std::size_t>(node)].child[bit];
        }
        tree.nodes[static_cast<std::size_t>(node)].sym =
            entries[static_cast<std::size_t>(i)].sym;
    }

    return tree;
}

auto explode_decompress(std::span<const std::byte> comp_data,
                        std::uint16_t flags,
                        std::uint64_t expected_size)
    -> std::vector<std::byte> {
    std::vector<std::byte> out;
    if (expected_size == 0) { return out; }
    out.reserve(static_cast<std::size_t>(expected_size));

    const bool eight_k_dict = (flags & IMPLODE_FLAG_8K_DICT) != 0;
    const bool three_trees = (flags & IMPLODE_FLAG_3_TREES) != 0;
    const int low_dist_bits = eight_k_dict ? 7 : 6;
    const std::size_t min_match = three_trees ? 3 : 2;

    LsbBitReader br(comp_data);
    SfTree lit_tree;
    if (three_trees) {
        lit_tree = read_sf_tree(br, 256);
    }
    const SfTree len_tree = read_sf_tree(br, 64);
    const SfTree dist_tree = read_sf_tree(br, 64);

    while (out.size() < expected_size) {
        std::uint32_t is_lit = 0;
        if (!br.read_bits(1, is_lit)) { break; }
        if (is_lit != 0) {
            int ch = 0;
            if (three_trees) {
                ch = lit_tree.decode(br);
            } else {
                std::uint32_t raw_byte = 0;
                if (!br.read_bits(8, raw_byte)) {
                    throw std::runtime_error("explode truncated literal");
                }
                ch = static_cast<int>(raw_byte);
            }
            out.push_back(static_cast<std::byte>(ch));
        } else {
            std::uint32_t low_dist = 0;
            if (!br.read_bits(low_dist_bits, low_dist)) {
                throw std::runtime_error("explode truncated distance");
            }
            const int high_dist = dist_tree.decode(br);
            const std::size_t dist =
                ((static_cast<std::size_t>(high_dist) << low_dist_bits) |
                 static_cast<std::size_t>(low_dist)) +
                1;

            const int len_code = len_tree.decode(br);
            std::size_t match_len = static_cast<std::size_t>(len_code) + min_match;
            if (len_code == 63) {
                std::uint32_t extra_len = 0;
                if (!br.read_bits(8, extra_len)) {
                    throw std::runtime_error("explode truncated match length");
                }
                match_len += static_cast<std::size_t>(extra_len);
            }

            for (std::size_t k = 0; k < match_len && out.size() < expected_size; ++k) {
                const std::size_t cur = out.size();
                out.push_back(cur < dist ? std::byte{0} : out[cur - dist]);
            }
        }
    }

    return out;
}

auto extract_entry_from(std::span<const std::byte> archive,
                        const ZipReadEntry& entry) -> std::vector<std::byte> {
    if (entry.is_directory) {
        return {};
    }
    if ((entry.flags & ZIP_FLAG_ENCRYPTED) != 0) {
        throw std::runtime_error("encrypted ZIP entry not supported: " + entry.name);
    }

    const std::byte* base = archive.data();

    // Read the local header at entry.local_header_offset to get the actual
    // compressed data offset (header may have different extra fields than CD).
    auto off = entry.local_header_offset;
    if (off + 30 > archive.size()) {
        throw std::runtime_error("local header offset out of bounds");
    }
    const std::byte* lh = base + off;
    if (read_u32(lh) != 0x04034b50u) {
        throw std::runtime_error("bad local header signature");
    }
    auto name_len = read_u16(lh + 26);
    auto extra_len = read_u16(lh + 28);
    auto data_offset = off + 30 + name_len + extra_len;
    if (data_offset + entry.compressed_size > archive.size()) {
        throw std::runtime_error("compressed data out of bounds");
    }

    std::span<const std::byte> comp_data{base + data_offset,
                                          entry.compressed_size};

    // Decompress.
    std::vector<std::byte> result;
    switch (entry.method) {
        case ZIP_METHOD_STORE:
        case ZIP_METHOD_DEFLATE:
        case ZIP_METHOD_ZSTD: {
            CompressedEntry ce;
            ce.codec = entry.codec;
            ce.data.assign(comp_data.begin(), comp_data.end());
            result = decompress(ce);
            break;
        }
        case ZIP_METHOD_SHRINK:
            result = unshrink_decompress(comp_data, entry.uncompressed_size);
            break;
        case ZIP_METHOD_IMPLODE:
            result = explode_decompress(comp_data, entry.flags, entry.uncompressed_size);
            break;
        default:
            throw std::runtime_error(
                "unsupported ZIP compression method " +
                std::to_string(entry.method) + ": " + entry.name);
    }

    // Verify CRC.
    auto actual_crc = crc32(std::span<const std::byte>{result.data(), result.size()});
    if (actual_crc != entry.crc32) {
        throw std::runtime_error("CRC mismatch: " + entry.name +
                                 " expected=" + std::to_string(entry.crc32) +
                                 " actual=" + std::to_string(actual_crc));
    }
    return result;
}

auto sanitize_rel_path(const std::string& raw,
                       std::filesystem::path& out_rel) -> bool {
    if (raw.empty()) { return false; }
    if (raw.find('\0') != std::string::npos) { return false; }
    std::string norm = raw;
    std::replace(norm.begin(), norm.end(), '\\', '/');
    if (norm.front() == '/') { return false; }
    if (norm.size() >= 2 && norm[1] == ':') { return false; }

    std::filesystem::path rel;
    std::size_t pos = 0;
    while (pos <= norm.size()) {
        std::size_t slash = norm.find('/', pos);
        if (slash == std::string::npos) { slash = norm.size(); }
        std::string comp = norm.substr(pos, slash - pos);
        pos = slash + 1;
        if (comp.empty() || comp == ".") { continue; }
        if (comp == "..") { return false; }
        rel /= std::filesystem::path(
            std::u8string_view(reinterpret_cast<const char8_t*>(comp.data()),
                               comp.size()));
    }
    out_rel = std::move(rel);
    return true;
}

}  // namespace

auto extract_entry(const std::string& archive_path, const ZipReadEntry& entry)
    -> std::vector<std::byte> {
    auto data = io::read_file(archive_path);
    return extract_entry_from(data, entry);
}

auto extract_all(const std::string& archive_path)
    -> std::vector<std::pair<std::string, std::vector<std::byte>>> {
    auto data = io::read_file(archive_path);
    auto entries = parse_zip_entries(data);
    const std::size_t n = entries.size();
    std::vector<std::pair<std::string, std::vector<std::byte>>> result(n);
    std::vector<std::exception_ptr> errors(n, nullptr);

    // Entries are independent, so decompress them on a thread pool. Errors are
    // captured per entry and rethrown on the calling thread.
    auto do_entry = [&](std::size_t i) {
        try {
            result[i].first = entries[i].name;
            result[i].second = extract_entry_from(data, entries[i]);
        } catch (...) {
            errors[i] = std::current_exception();
        }
    };

    unsigned hw = std::thread::hardware_concurrency();
    if (hw == 0) hw = 1;
    if (n < 2 || hw <= 1) {
        for (std::size_t i = 0; i < n; ++i) do_entry(i);
    } else {
        const unsigned nw =
            std::min<unsigned>(hw, static_cast<unsigned>(n));
        std::atomic<std::size_t> next{0};
        std::vector<std::thread> workers;
        workers.reserve(nw);
        for (unsigned t = 0; t < nw; ++t) {
            workers.emplace_back([&]() {
                for (;;) {
                    std::size_t i = next.fetch_add(1, std::memory_order_relaxed);
                    if (i >= n) break;
                    do_entry(i);
                }
            });
        }
        for (auto& worker : workers) worker.join();
    }

    for (const auto& e : errors) {
        if (e) std::rethrow_exception(e);
    }
    return result;
}

auto extract_to_directory(const std::string& archive_path,
                          const std::filesystem::path& dest_dir) -> bool {
    try {
        std::error_code ec;
        std::filesystem::create_directories(dest_dir, ec);
        if (ec) { return false; }
        const auto base_canon = std::filesystem::weakly_canonical(dest_dir, ec);
        if (ec) { return false; }

        auto data = io::read_file(archive_path);
        auto entries = parse_zip_entries(data);
        const std::size_t n = entries.size();

        std::vector<std::filesystem::path> targets(n);
        for (std::size_t i = 0; i < n; ++i) {
            std::filesystem::path rel;
            if (!sanitize_rel_path(entries[i].name, rel)) { return false; }
            if (rel.empty()) {
                if (!entries[i].is_directory) { return false; }
                targets[i] = base_canon;
                continue;
            }
            const auto candidate = base_canon / rel;
            const auto rel_check = candidate.lexically_relative(base_canon);
            if (rel_check.empty()) { return false; }
            for (const auto& part : rel_check) {
                if (part == "..") { return false; }
            }
            targets[i] = candidate;
        }

        std::vector<std::vector<std::byte>> buffers(n);
        std::vector<std::exception_ptr> errors(n, nullptr);
        auto do_entry = [&](std::size_t i) {
            try {
                if (!entries[i].is_directory) {
                    buffers[i] = extract_entry_from(data, entries[i]);
                }
            } catch (...) {
                errors[i] = std::current_exception();
            }
        };

        unsigned hw = std::thread::hardware_concurrency();
        if (hw == 0) { hw = 1; }
        if (n < 2 || hw <= 1) {
            for (std::size_t i = 0; i < n; ++i) { do_entry(i); }
        } else {
            const unsigned nw = std::min<unsigned>(hw, static_cast<unsigned>(n));
            std::atomic<std::size_t> next{0};
            std::vector<std::thread> workers;
            workers.reserve(nw);
            for (unsigned t = 0; t < nw; ++t) {
                workers.emplace_back([&]() {
                    for (;;) {
                        const std::size_t i =
                            next.fetch_add(1, std::memory_order_relaxed);
                        if (i >= n) { break; }
                        do_entry(i);
                    }
                });
            }
            for (auto& worker : workers) { worker.join(); }
        }

        for (const auto& e : errors) {
            if (e) { return false; }
        }

        for (std::size_t i = 0; i < n; ++i) {
            if (entries[i].is_directory) {
                std::filesystem::create_directories(targets[i], ec);
                if (ec) { return false; }
                continue;
            }
            const auto parent = targets[i].parent_path();
            if (!parent.empty()) {
                std::filesystem::create_directories(parent, ec);
                if (ec) { return false; }
            }
            std::ofstream ofs(targets[i], std::ios::binary | std::ios::trunc);
            if (!ofs) { return false; }
            if (!buffers[i].empty()) {
                ofs.write(reinterpret_cast<const char*>(buffers[i].data()),
                          static_cast<std::streamsize>(buffers[i].size()));
            }
            if (!ofs.good()) { return false; }
        }
        return true;
    } catch (...) {
        return false;
    }
}

}  // namespace fzip
