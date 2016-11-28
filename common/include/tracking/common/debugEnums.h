//
// Created by bontius on 31/03/16.
//

#ifndef TRACKVIDEO_COMMON_DEBUGENUMS_H
#define TRACKVIDEO_COMMON_DEBUGENUMS_H

namespace tracking {
  enum SILENCE {
      NONE,     //!< show everything
      QUICK,    //!< show, but do not wait for input for most windows
      QUICKER,  //!< show, but do not wait for input ever
      TOTAL};   //!< show and wait for input at every window
} //...ns tracking

#endif //TRACKVIDEO_COMMON_DEBUGENUMS_H
