

#ifndef __WORLDMAGNETICMODEL_H_INCLUDED
#define __WORLDMAGNETICMODEL_H_INCLUDED


typedef struct WorldMagModelStruct_s{
    uint32_t timeOfLastSoln;
    bool validSoln;
    float decl_rad;
} WorldMagModelStruct;



#endif /* MAGALIGN_H */
