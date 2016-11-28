//
// Created by bontius on 02/03/16.
//

#ifndef TRACKVIDEO_DEFINES_H
#define TRACKVIDEO_DEFINES_H

#define EXPOSE_GET_CONST(TYPE, NAME, VARNAME) \
    inline TYPE const& get##NAME(void) const { return VARNAME; }

#define EXPOSE_GET(TYPE, NAME, VARNAME) \
    inline TYPE const& get##NAME(void) const { return VARNAME; } \
    inline TYPE      & get##NAME(void)       { return VARNAME; }

#define EXPOSE(TYPE, NAME, VARNAME) \
    inline const TYPE & get##NAME(void) const { return VARNAME; } \
    inline void set##NAME(const TYPE & val) { VARNAME = val; }

#define EXPOSE_DIRTY(TYPE, NAME, VARNAME) \
    const TYPE & get##NAME(void) const { return VARNAME; } \
    void set##NAME(const TYPE & val) { VARNAME = val; _dirty = true; }

#define EXPOSE_SHARED_PTR(TYPE, NAME, VARNAME) \
    std::shared_ptr<const TYPE> get##NAME(void) const { return VARNAME; } \
    std::shared_ptr<TYPE> get##NAME(void) { return VARNAME; } \
    void set##NAME(std::shared_ptr<TYPE> val) { VARNAME = val; }

#endif //TRACKVIDEO_DEFINES_H
