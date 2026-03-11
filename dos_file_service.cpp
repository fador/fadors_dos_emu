void DOS::handleFileService() {
  uint8_t ah = m_cpu.getReg8(cpu::AH);
  if (ah == 0x3C) { // Create or Truncate File
    uint16_t ds = m_cpu.getSegReg(cpu::DS);
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    uint32_t nameAddr = (ds << 4) + dx;
    std::string filename = readFilename(nameAddr);
    std::string hostPath = resolvePath(filename);

    auto fh = std::make_unique<FileHandle>();
    fh->path = hostPath;
    fh->stream.open(hostPath, std::ios::out | std::ios::binary | std::ios::trunc | std::ios::in);
    
    if (fh->stream.is_open()) {
      m_fileHandles.push_back(std::move(fh));
      m_cpu.setReg16(cpu::AX, m_fileHandles.size() - 1 + 5); // DOS handles start at 5
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Created file '", hostPath, "' handle=", m_fileHandles.size() - 1 + 5);
    } else {
      m_cpu.setReg16(cpu::AX, 0x03); // Path not found
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_ERROR("DOS: Failed to create file '", hostPath, "'");
    }
  } else if (ah == 0x3D) { // Open Existing File
    uint16_t ds = m_cpu.getSegReg(cpu::DS);
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    uint8_t accessMode = m_cpu.getReg8(cpu::AL) & 0x03; // 0=Read, 1=Write, 2=Read/Write
    uint32_t nameAddr = (ds << 4) + dx;
    std::string filename = readFilename(nameAddr);
    std::string hostPath = resolvePath(filename);

    auto fh = std::make_unique<FileHandle>();
    fh->path = hostPath;
    
    std::ios::openmode mode = std::ios::binary;
    if (accessMode == 0) mode |= std::ios::in;
    else if (accessMode == 1) mode |= std::ios::out | std::ios::app;
    else mode |= std::ios::in | std::ios::out;
    
    fh->stream.open(hostPath, mode);
    
    if (fh->stream.is_open()) {
      m_fileHandles.push_back(std::move(fh));
      m_cpu.setReg16(cpu::AX, m_fileHandles.size() - 1 + 5); 
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Opened file '", hostPath, "' handle=", m_fileHandles.size() - 1 + 5);
    } else {
      m_cpu.setReg16(cpu::AX, 0x02); // File not found
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_ERROR("DOS: Failed to open file '", hostPath, "'");
    }
  } else if (ah == 0x3E) { // Close File
    uint16_t handle = m_cpu.getReg16(cpu::BX);
    if (handle >= 5 && handle - 5 < m_fileHandles.size() && m_fileHandles[handle - 5]) {
      m_fileHandles[handle - 5]->stream.close();
      m_fileHandles[handle - 5].reset(); // Free slot
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Closed file handle=", handle);
    } else if (handle < 5) { // Standard handles
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Closed standard handle=", handle);
    } else {
      m_cpu.setReg16(cpu::AX, 0x06); // Invalid handle
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
  } else if (ah == 0x3F) { // Read from File or Device
    uint16_t handle = m_cpu.getReg16(cpu::BX);
    uint16_t bytesToRead = m_cpu.getReg16(cpu::CX);
    uint16_t ds = m_cpu.getSegReg(cpu::DS);
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    uint32_t bufAddr = (ds << 4) + dx;

    if (handle >= 5 && handle - 5 < m_fileHandles.size() && m_fileHandles[handle - 5]) {
      auto& fh = m_fileHandles[handle - 5];
      std::vector<char> buf(bytesToRead);
      fh->stream.read(buf.data(), bytesToRead);
      std::streamsize bytesRead = fh->stream.gcount();
      
      for (std::streamsize i = 0; i < bytesRead; ++i) {
        m_memory.write8(bufAddr + i, buf[i]);
      }
      
      m_cpu.setReg16(cpu::AX, static_cast<uint16_t>(bytesRead));
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Read ", bytesRead, " bytes from handle=", handle);
    } else if (handle == 0) { // STDIN
      m_cpu.setReg16(cpu::AX, 0); // Not fully implemented
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else {
      m_cpu.setReg16(cpu::AX, 0x06); // Invalid handle
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
  } else if (ah == 0x40) { // Write to File or Device
    uint16_t handle = m_cpu.getReg16(cpu::BX);
    uint16_t bytesToWrite = m_cpu.getReg16(cpu::CX);
    uint16_t ds = m_cpu.getSegReg(cpu::DS);
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    uint32_t bufAddr = (ds << 4) + dx;

    if (handle == 1 || handle == 2) { // STDOUT / STDERR
      for (uint16_t i = 0; i < bytesToWrite; ++i) {
        writeCharToVRAM(m_memory.read8(bufAddr + i));
      }
      m_cpu.setReg16(cpu::AX, bytesToWrite);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else if (handle >= 5 && handle - 5 < m_fileHandles.size() && m_fileHandles[handle - 5]) {
      auto& fh = m_fileHandles[handle - 5];
      std::vector<char> buf(bytesToWrite);
      for (uint16_t i = 0; i < bytesToWrite; ++i) {
        buf[i] = m_memory.read8(bufAddr + i);
      }
      fh->stream.write(buf.data(), bytesToWrite);
      
      if (fh->stream.good()) {
        m_cpu.setReg16(cpu::AX, bytesToWrite);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
        LOG_DOS("DOS: Wrote ", bytesToWrite, " bytes to handle=", handle);
      } else {
        m_cpu.setReg16(cpu::AX, 0x1D); // Write fault
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      }
    } else {
      m_cpu.setReg16(cpu::AX, 0x06); // Invalid handle
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
  } else if (ah == 0x42) { // Move File Pointer (Seek)
    uint16_t handle = m_cpu.getReg16(cpu::BX);
    uint8_t method = m_cpu.getReg8(cpu::AL);
    uint16_t cx = m_cpu.getReg16(cpu::CX); // High word of offset
    uint16_t dx = m_cpu.getReg16(cpu::DX); // Low word of offset
    int32_t offset = (cx << 16) | dx;

    if (handle >= 5 && handle - 5 < m_fileHandles.size() && m_fileHandles[handle - 5]) {
      auto& fh = m_fileHandles[handle - 5];
      std::ios_base::seekdir dir;
      if (method == 0) dir = std::ios::beg;
      else if (method == 1) dir = std::ios::cur;
      else if (method == 2) dir = std::ios::end;
      else {
        m_cpu.setReg16(cpu::AX, 0x01); // Invalid function
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
        return;
      }
      
      fh->stream.clear(); // Clear EOF flags
      fh->stream.seekg(offset, dir);
      fh->stream.seekp(offset, dir);
      
      uint32_t newPos = fh->stream.tellg();
      m_cpu.setReg16(cpu::DX, newPos >> 16);
      m_cpu.setReg16(cpu::AX, newPos & 0xFFFF);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Seeked handle=", handle, " to pos=", newPos);
    } else {
      m_cpu.setReg16(cpu::AX, 0x06); // Invalid handle
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
  } else if (ah == 0x43) { // Get/Set File Attributes
    uint16_t ds = m_cpu.getSegReg(cpu::DS);
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    uint8_t al = m_cpu.getReg8(cpu::AL);
    uint32_t nameAddr = (ds << 4) + dx;
    std::string filename = readFilename(nameAddr);
    std::string hostPath = resolvePath(filename);
    
    std::error_code ec;
    if (al == 0x00) { // Get
      if (fs::exists(hostPath, ec)) {
        uint16_t attr = 0x20; // Archive
        if (fs::is_directory(hostPath, ec)) attr = 0x10;
        m_cpu.setReg16(cpu::CX, attr);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
        LOG_DOS("DOS: Get attributes for '", hostPath, "'");
      } else {
        m_cpu.setReg16(cpu::AX, 0x02); // File not found
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      }
    } else if (al == 0x01) { // Set (Stubbed)
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Set attributes for '", hostPath, "' (stubbed)");
    } else {
      m_cpu.setReg16(cpu::AX, 0x01); // Invalid function
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
  }
}
