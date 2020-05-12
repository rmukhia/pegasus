#ifndef _PEGASUSPACKET_H
#define _PEGASUSPACKET_H


#include "PegasusVariables.h"

class PegasusPacket {
  private:
    char m_buffer[MAX_PACKET_SIZE];

    int m_len;


  public:
    PegasusPacket(const char* buffer, int len);

    PegasusPacket();

    inline char const (* Get_m_buffer() const);

    inline const int Get_m_len() const;

};
inline char const (* PegasusPacket::Get_m_buffer() const) {
  return m_buffer;
}

inline const int PegasusPacket::Get_m_len() const {
  return m_len;
}

#endif
