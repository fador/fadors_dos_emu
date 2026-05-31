# DOS 6.22 Command Prompt Emulator — Implementation Plan

## Overview

Transform the emulator from a "load-and-run single program" tool into a full DOS 6.22-like shell environment with drive mounting, internal commands, external program execution, and BAT scripting. The shell is implemented natively in C++ (no real COMMAND.COM binary required).

---

## Architecture

### New Files

| File | Purpose |
|---|---|
| `src/hw/DosShell.hpp/cpp` | COMMAND.COM-equivalent shell: prompt, input loop, command parsing, internal command dispatch, external program execution |
| `src/hw/DriveManager.hpp/cpp` | Virtual drive table: mount points, per-drive CWD, path resolution |
| `src/hw/BatchInterpreter.hpp/cpp` | BAT file parser and executor: `%1`-`%9`, `%VAR%`, `IF`, `GOTO`, `FOR`, `CALL`, `ECHO`, `PAUSE`, `SHIFT` |
| `tests/test_dos_shell.cpp` | Unit tests for shell commands, path parsing, drive management |
| `tests/test_batch.cpp` | Unit tests for BAT scripting features |
| `tests/test_drive_manager.cpp` | Unit tests for drive mounting, path resolution, per-drive CWD |

### Modified Files

| File | Changes |
|---|---|
| `src/hw/DOS.hpp` | Add `DriveManager` reference; add per-drive CWD state; add INT 21h AH=0Ah buffered input implementation |
| `src/hw/DOS.cpp` | Version → 6.22; implement AH=0Ah properly; update `resolvePath()` to use `DriveManager`; add `m_driveManager` |
| `src/main.cpp` | New CLI flags (`--mount=`, directory arg); default to shell when no program given; wire `DosShell` into the execution loop |
| `TODO.md` | Add Phase 12 entries |

---

## Component 1: Drive Manager (`DriveManager`)

### Data Model

```
struct MountEntry {
    char letter;              // 'A'-'Z'
    std::string hostPath;     // Absolute host directory path
    std::string volumeLabel;  // "MS-DOS_622" or user-specified
    bool readOnly;
    bool isVirtual;           // true for Z: (built-in tools drive)
};

class DriveManager {
    std::array<std::string, 26> m_mounts;        // host path per drive letter (empty = unmounted)
    std::array<std::string, 26> m_currentDir;    // per-drive CWD (DOS-style, e.g. "GAMES\DOOM")
    std::string m_volumeLabels[26];
    uint8_t m_currentDrive = 25;                  // default Z: (index 25)

    // Core operations
    bool mount(char letter, const std::string& hostPath, const std::string& label = "");
    bool unmount(char letter);
    bool isMounted(char letter) const;
    std::string resolvePath(const std::string& dosPath) const;
    // Returns host absolute path for a DOS path like "C:\GAMES\DOOM.EXE"
    // Handles: drive letter prefix, relative paths, "..", "."
    std::string getHostPath(char letter) const;
    void setCurrentDrive(char letter);
    char getCurrentDrive() const;
    std::string getCurrentDir(char letter) const;
    bool setCurrentDir(char letter, const std::string& dosDir);
    std::string getDosCwd() const; // full "C:\GAMES\DOOM" style
    int getTotalMounted() const;
};
```

### Path Resolution Algorithm

1. Parse drive letter from path (e.g., `C:\FOO\BAR` → drive `C`, remainder `\FOO\BAR`)
2. If no drive letter, use current drive
3. If path starts with `\`, it's absolute relative to drive root → append to mount host path
4. Otherwise, it's relative → append to per-drive CWD then to mount host path
5. Normalize: collapse `..`, `.`, convert `/` to `\`, uppercase
6. Verify the resolved path doesn't escape the mount root (security)

### Default Mounts

- `Z:` → emulator's built-in tools directory (virtual, read-only; contains HELP.COM, etc.)
- When a directory is specified on the CLI: mount it as `C:`
- When `--mount=X:/path` is specified: mount accordingly

---

## Component 2: DOS Shell (`DosShell`)

### Shell Loop

```
1. Display prompt: "C:\GAMES>" (or "Z:\>" if no C: mounted)
2. Read a line of input via INT 21h AH=0Ah (buffered input)
3. Parse command line:
   - Split into command + arguments (respecting quotes)
   - Handle redirection: >, >>, < (future)
   - Handle pipe: | (future / optional)
4. If command matches an internal command → dispatch
5. Else → search for external .COM/.EXE/.BAT in PATH and current dir
6. If found .BAT → invoke BatchInterpreter
7. If found .COM/.EXE → invoke via INT 21h AH=4Bh (Exec)
8. If not found → display "Bad command or file name"
9. Loop back to 1
```

### Internal Commands (DOS 6.22 COMMAND.COM)

| Command | Description | Key Features |
|---|---|---|
| `DIR` | Directory listing | `/W` wide, `/P` paged, `/S` recursive, `/A:attr` filter, `/O:sort`, `*`/`?` wildcards, display size/date/time/attr |
| `CD` / `CHDIR` | Change directory | `CD ..`, `CD \`, `CD path`, `CD` alone prints CWD |
| `MD` / `MKDIR` | Make directory | |
| `RD` / `RMDIR` | Remove directory | Must be empty |
| `COPY` | Copy files | `COPY A B`, `COPY *.TXT dest`, `COPY A+B C` concatenate |
| `DEL` / `ERASE` | Delete files | `DEL *.TXT`, `/P` prompt |
| `REN` / `RENAME` | Rename file | |
| `TYPE` | Display file contents | |
| `ECHO` | Print text / toggle echo | `ECHO ON/OFF`, `ECHO text`, `ECHO.` (blank line) |
| `SET` | Environment variables | `SET`, `SET VAR=value`, `SET VAR=` (delete) |
| `PATH` | Set/search path | `PATH`, `PATH C:\;D:\UTILS` |
| `PROMPT` | Set prompt string | `$P$G` default, `$D`, `$T`, `$_`, `$$` |
| `CLS` | Clear screen | INT 10h AH=00h mode set + cursor reset |
| `DATE` | Display/set date | |
| `TIME` | Display/set time | |
| `VER` | Display DOS version | "MS-DOS Version 6.22" |
| `VOL` | Display volume label | |
| `BREAK` | Ctrl-Break check | ON/OFF |
| `VERIFY` | Verify writes | ON/OFF (stub) |
| `EXIT` | Exit shell / terminate | |
| `COLOR` | Set text color | (optional, DOS 6.22 didn't have this but useful) |
| `MOUNT` | Mount a host directory | `MOUNT D /home/user/games`, `MOUNT` lists mounts |
| `UNMOUNT` | Unmount a drive | |
| `HELP` | List available commands | |
| `TREE` | Display directory tree | |
| `MOVE` | Move/rename files | |
| `XCOPY` | Extended copy | `/S` recursive, `/E` include empty |
| `FIND` | Find text in files | `FIND "text" file` |
| `MORE` | Page through output | |
| `SORT` | Sort input | |
| `MEM` | Display memory usage | Show conventional, extended, free |
| `CHKDSK` | Check disk | Display size/free (simulated) |

### External Program Execution

When the user types a command that doesn't match an internal command:
1. Search current directory for `cmd.COM`, `cmd.EXE`, `cmd.BAT`
2. Search each directory in `%PATH%` for the same
3. If found `.COM`/`.EXE`: load via `ProgramLoader`, create PSP with command tail, execute via the existing INT 21h AH=4Bh mechanism
4. After execution (INT 21h AH=4Ch / INT 20h), return to shell prompt
5. Preserve shell state (current drive, CWD, environment) across program execution

---

## Component 3: BAT Interpreter (`BatchInterpreter`)

### Features (DOS 6.22 compatible)

| Feature | Syntax | Description |
|------|---|---|
| Parameters | `%0`-`%9` | `%0` = batch file name, `%1`-`%9` = arguments |
| Shift | `SHIFT` | Shift parameters left (exposes `%10`+ via `%1`) |
| Variable expansion | `%VAR%` | Environment variable substitution |
| ECHO | `ECHO text` / `ECHO ON/OFF` / `ECHO OFF` | Print or toggle command echoing |
| @ECHO OFF | `@ECHO OFF` | Suppress display of the command itself |
| GOTO | `GOTO label` / `:label` | Jump to label |
| IF | `IF [NOT] EXIST file cmd` / `IF [NOT] string1==string2 cmd` / `IF [NOT] ERRORLEVEL n cmd` | Conditional execution |
| FOR | `FOR %%var IN (set) DO cmd` | Loop over a set |
| CALL | `CALL batch.bat args` / `CALL :label` | Invoke another batch file or subroutine |
| PAUSE | `PAUSE [message]` | Wait for keypress |
| REM | `REM comment` | Comment |
| CHOICE | `CHOICE /C:YN prompt` | User choice prompt (DOS 6.22 feature) |
| ERRORLEVEL | `%ERRORLEVEL%` | Exit code of last program |

### Execution Model

1. Read BAT file line by line (support `LF` and `CRLF`)
2. Pre-process: expand `%variables%`, substitute `%1`-`%9`
3. If `@` prefix, suppress echo for this line
4. If `ECHO OFF` is active, don't print the command line itself (unless `ECHO ON`)
5. Execute the line (internal command, external program, or batch control statement)
6. Track `ERRORLEVEL` from each executed command
7. Support `GOTO` by scanning for `:label` lines (forward and backward)
8. Support `CALL :label` by pushing current line offset onto a call stack
9. `EXIT` in a batch file returns to the caller (not the shell)

---

## Component 4: INT 21h Updates

### AH=0Ah — Buffered Keyboard Input (Critical for Shell)

**Current**: Stubbed, returns 0 chars + CR immediately.

**Implementation**:
1. Read `DS:DX` → first byte = max chars (including CR)
2. Read characters one at a time via keyboard controller:
   - Display each character via `writeCharToVRAM()` (echo)
   - Handle backspace (erase last char, move cursor back)
   - Handle Enter (append CR, done)
   - Handle Ctrl-C (invoke INT 23h if break checking enabled)
   - Respect max length
3. Write actual char count at `DS:DX+1`
4. Write characters starting at `DS:DX+2`
5. Write CR at end

### AH=30h — DOS Version

Change from 5.0 → 6.22:
```
AL = 6 (major), AH = 22 (minor)
BX = 0xFF00 (OEM)
CX = 0x0000
```

### AH=0Eh — Select Drive

Update to return total number of **mounted** drives (not hardcoded 5).

### AH=36h — Get Free Disk Space

Update to return realistic values based on the host mount's actual free space (`std::filesystem::space()`).

### AH=19h — Get Current Drive

Return the `DriveManager`'s current drive.

### AH=3Bh — CHDIR

Update to use `DriveManager::setCurrentDir()` with per-drive tracking.

### AH=47h — Get Current Directory

Read from `DriveManager::getCurrentDir()` for the specified drive.

### AH=4Eh/4Fh — FindFirst/FindNext

Update to resolve paths through `DriveManager`.

### All file operations (3Ch, 3Dh, 41h, 43h, 56h, etc.)

Update `resolvePath()` to delegate to `DriveManager::resolvePath()`.

---

## Component 5: CLI Changes (`main.cpp`)

### New Argument Parsing

```
fadors_emu [OPTIONS] [program|directory] [args...]

If no arguments: start DOS prompt at Z:\>
If first non-flag arg is a directory: mount as C:, start DOS prompt at C:\>
If first non-flag arg is .COM/.EXE: run directly (existing behavior)
```

### New Flags

| Flag | Description |
|---|---|
| `--mount=L:/path` | Mount host directory at drive letter L (repeatable) |
| `--no-shell` | Skip shell, run program directly (explicit opt-in to old behavior) |

### Execution Flow Changes

```
main():
  ... existing hardware init ...

  DriveManager driveManager;
  // Z: is always the built-in virtual drive
  driveManager.mount('Z', getExecutableDir() + "/tools", "FADOR_UTIL", true);

  // CLI mounts
  for (auto& m : cliMounts)
    driveManager.mount(m.letter, m.hostPath, m.label);

  dos.setDriveManager(&driveManager);

  if (programPath is a directory) {
    driveManager.mount('C', absolute(programPath), "MS-DOS_622");
    DosShell shell(dos, driveManager, decoder, loader, ...);
    shell.run();  // enters the shell loop
    return 0;
  }

  if (programPath is a .COM/.EXE) {
    // existing direct execution (but now uses DriveManager for paths)
    ...
  }

  if (no arguments) {
    DosShell shell(dos, driveManager, decoder, loader, ...);
    shell.run();
    return 0;
  }
```

---

## Implementation Order

### Phase 1: Drive Manager Foundation
1. Create `DriveManager` class with mount/unmount, per-drive CWD, path resolution
2. Wire into `DOS` class: update all path resolution to go through `DriveManager`
3. Update `resolvePath()`, `dosToHostPath()`, `hostToDosPath()` in DOS.cpp
4. Update INT 21h handlers: AH=0Eh, 19h, 3Bh, 47h, 36h
5. Unit tests: `test_drive_manager.cpp`

### Phase 2: Shell Core
1. Create `DosShell` class with prompt display, input loop, command parsing
2. Implement INT 21h AH=0Ah (buffered keyboard input) — critical for interactive use
3. Implement `EXIT` and `VER` commands
4. Wire into `main.cpp` as default when no program given
5. Update DOS version to 6.22

### Phase 3: Internal Commands — File System
1. `DIR` (with wildcards, /W, /P, /S, /O, /A)
2. `CD`, `MD`, `RD`
3. `COPY`, `DEL`, `REN`, `TYPE`
4. `TREE`, `MOVE`, `XCOPY`

### Phase 4: Internal Commands — System
1. `SET`, `PATH`, `PROMPT`, `CLS`
2. `DATE`, `TIME`, `VER`, `VOL`
3. `ECHO`, `BREAK`, `VERIFY`
4. `MEM`, `CHKDSK`, `HELP`
5. `MOUNT`, `UNMOUNT`
6. `FIND`, `MORE`, `SORT`

### Phase 5: External Program Execution
1. Search PATH for external commands
2. Load and execute .COM/.EXE via existing ProgramLoader + INT 21h AH=4Bh
3. Handle return to shell after program terminates
4. Preserve shell state across executions

### Phase 6: BAT Scripting
1. Create `BatchInterpreter` class
2. Line reading, `%1`-`%9` substitution, `%VAR%` expansion
3. `ECHO`, `ECHO OFF`, `@`, `REM`, `PAUSE`
4. `GOTO` / `:labels`, `IF`, `FOR`
5. `CALL`, `SHIFT`, `ERRORLEVEL`
6. `CHOICE` (DOS 6.22 specific)
7. Unit tests: `test_batch.cpp`

### Phase 7: Polish & Integration
1. Tab completion (optional)
2. Command history (optional)
3. Redirection (`>`, `>>`, `<`) and piping (`|`) — optional
4. Update `TODO.md`
5. Full integration test: mount → navigate → run program → batch file

---

## Key Design Decisions

1. **Z: is the permanent utility drive** — contains HELP and any built-in tools. Cannot be unmounted.
2. **No FAT emulation** — all file operations go through the host filesystem via `std::filesystem`. This matches the existing approach and avoids the complexity of a virtual disk format.
3. **Shell uses INT 21h for I/O** — the shell reads input via INT 21h AH=0Ah and writes output via AH=02h/09h. This ensures compatibility and proper VRAM rendering.
4. **Per-drive CWD** — each drive letter has its own current directory, matching real DOS behavior. Stored in `DriveManager`, backed up/restored by `ProcessState` during EXEC.
5. **PATH search order** — current directory first, then each directory in `%PATH%`, matching DOS convention.
6. **BAT files execute line-by-line** — no pre-parsing of the entire file (matches DOS behavior where BAT files can modify themselves).

---

## Estimated Scope

| Component | New LOC (approx) | New Files |
|---|---|---|
| DriveManager | ~300 | 2 (hpp/cpp) |
| DosShell | ~1200 | 2 |
| BatchInterpreter | ~600 | 2 |
| DOS.cpp updates | ~200 diff | 0 (modify) |
| main.cpp updates | ~100 diff | 0 (modify) |
| Tests | ~800 | 3 |
| **Total** | **~3200** | **7 new, 2 modified** |
