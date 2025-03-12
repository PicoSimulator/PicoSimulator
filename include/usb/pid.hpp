#pragma once

#define EXPAND_PID(pid) (((pid) << 4) | ~(pid))

#define ENUM_TOKEN_PIDS(o) \
  o(OUT  , 0b0001) \
  o(IN   , 0b1001) \
  o(SOF  , 0b0101) \
  o(SETUP, 0b1101) \

#define ENUM_DATA_PIDS(o) \
  o(DATA0, 0b0011) \
  o(DATA1, 0b1011) \
  o(DATA2, 0b0111) \
  o(MDATA, 0b1111)

#define ENUM_HANDSHAKE_PIDS(o) \
  o(ACK  , 0b0010) \
  o(NAK  , 0b1010) \
  o(NYET , 0b0110) \
  o(STALL, 0b1110)

#define ENUM_SPECIAL_PIDS(o) \
  o(SPLIT, 0b1000) \
  o(PING , 0b0100) \
  o(PRE  , 0b1100) \
  o(ERR  , 0b1100)

#define EVAL_PID(name, val) name = EXPAND_PID(val),

namespace USB {

  enum class PID {
    ENUM_TOKEN_PIDS(EVAL_PID)
    ENUM_DATA_PIDS(EVAL_PID)
    ENUM_HANDSHAKE_PIDS(EVAL_PID)
    ENUM_SPECIAL_PIDS(EVAL_PID)
  };
  enum class TokenPID {
    ENUM_TOKEN_PIDS(EVAL_PID)
  };
  enum class DataPID {
    ENUM_DATA_PIDS(EVAL_PID)
  };
  enum class HandshakePID {
    ENUM_HANDSHAKE_PIDS(EVAL_PID)
  };

}