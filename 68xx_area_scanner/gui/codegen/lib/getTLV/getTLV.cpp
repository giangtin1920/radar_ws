//
// File: getTLV.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 27-Nov-2021 11:15:16
//

// Include Files
#include "getTLV.h"
#include "getTLV_types.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : const double framePacket[128]
//                double *idx
//                struct0_T *tlv
// Return Type  : void
//
void getTLV(const double framePacket[128], double *idx, struct0_T *tlv)
{
  double idxEnd;
  double idxStart;
  double y;
  int tmp_data[128];
  int i;
  int i1;
  int loop_ub;

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //
  //       (C) Copyright 2016 Texas Instruments, Inc.
  //
  //   Redistribution and use in source and binary forms, with or without
  //   modification, are permitted provided that the following conditions
  //   are met:
  //
  //     Redistributions of source code must retain the above copyright
  //     notice, this list of conditions and the following disclaimer.
  //
  //     Redistributions in binary form must reproduce the above copyright
  //     notice, this list of conditions and the following disclaimer in the
  //     documentation and/or other materials provided with the
  //     distribution.
  //
  //     Neither the name of Texas Instruments Incorporated nor the names of
  //     its contributors may be used to endorse or promote products derived
  //     from this software without specific prior written permission.
  //
  //   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  //   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  //   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  //   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  //   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  //   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  //   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  //   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  //   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  //
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //  framePacket = framePacket';
  //  size of the parameters; 0 = variable/unknown
  //  get fieldnames of frame header struct
  //  subindices for parsing to struct fields
  //  parse to struct fields
  if (*idx + 1.0 > ((*idx + 1.0) + 4.0) - 1.0) {
    i = 0;
    i1 = 0;
  } else {
    i = static_cast<int>(*idx + 1.0) - 1;
    i1 = static_cast<int>(((*idx + 1.0) + 4.0) - 1.0);
  }

  loop_ub = i1 - i;
  for (i1 = 0; i1 < loop_ub; i1++) {
    tmp_data[i1] = i + i1;
  }

  tlv->type = ((framePacket[tmp_data[0]] + framePacket[tmp_data[1]] * 256.0) +
               framePacket[tmp_data[2]] * 65536.0) + framePacket[tmp_data[3]] *
    1.6777216E+7;

  //  subindices for parsing to struct fields
  idxEnd = (((((*idx + 1.0) + 4.0) - 1.0) + 1.0) + 4.0) - 1.0;

  //  parse to struct fields
  if ((((*idx + 1.0) + 4.0) - 1.0) + 1.0 > (((((*idx + 1.0) + 4.0) - 1.0) + 1.0)
       + 4.0) - 1.0) {
    i = 0;
    i1 = 0;
  } else {
    i = static_cast<int>((((*idx + 1.0) + 4.0) - 1.0) + 1.0) - 1;
    i1 = static_cast<int>((((((*idx + 1.0) + 4.0) - 1.0) + 1.0) + 4.0) - 1.0);
  }

  loop_ub = i1 - i;
  for (i1 = 0; i1 < loop_ub; i1++) {
    tmp_data[i1] = i + i1;
  }

  y = ((framePacket[tmp_data[0]] + framePacket[tmp_data[1]] * 256.0) +
       framePacket[tmp_data[2]] * 65536.0) + framePacket[tmp_data[3]] *
    1.6777216E+7;
  tlv->length = y;

  //  subindices for parsing to struct fields
  idxStart = ((((((*idx + 1.0) + 4.0) - 1.0) + 1.0) + 4.0) - 1.0) + 1.0;
  *idx = (((((((*idx + 1.0) + 4.0) - 1.0) + 1.0) + 4.0) - 1.0) + 1.0) - 1.0;

  //  parse to struct fields
  if (y != 0.0) {
    //  have payload
    //  idxEnd = idxStart+tlv.length-1;
    *idx = 13.0;
    if (idxStart > 13.0) {
      i = -1;
      i1 = -1;
    } else {
      i = static_cast<int>(idxEnd + 1.0) - 2;
      i1 = 12;
    }

    loop_ub = static_cast<signed char>(i1 - i);
    tlv->payload.size[0] = loop_ub;
    tlv->payload.size[1] = 1;
    for (i1 = 0; i1 < loop_ub; i1++) {
      tlv->payload.data[i1] = framePacket[(i + i1) + 1];
    }
  } else {
    //  no payload
    tlv->payload.size[0] = 0;
    tlv->payload.size[1] = 0;
  }
}

//
// File trailer for getTLV.cpp
//
// [EOF]
//
