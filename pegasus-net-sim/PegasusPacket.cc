
#include "PegasusPacket.h"

#include <cstring>

PegasusPacket::PegasusPacket(const char* buffer, int len) {
  std::memcpy(m_buffer, buffer, len);
  m_len = len;
}

PegasusPacket::PegasusPacket() {
  m_len = 0;
}
