#include <vector>
#include <iostream>
#include <cstdlib>
#include <algorithm>
using namespace std;

// uint16_t dataLen = 1731;
// uint8_t raw_data[] = {1,1,1,2,1,4,3,6,5,8,7,4,0,5,3,192,6,0,0,67,104,10,0,21,0,0,0,95,100,108,215,27,0,0,0,6,0,0,0,0,0,0,0,1,0,0,0,176,1,0,0,82,81,192,189,44,207,62,63,0,0,0,0,0,0,0,0,91,188,14,62,122,146,97,63,0,0,0,0,0,0,0,0,60,76,180,61,155,86,111,63,0,0,0,0,0,0,0,0,13,80,189,189,36,78,123,63,0,0,0,0,0,0,0,0,232,55,132,189,188,245,131,63,0,0,0,0,0,0,0,0,124,91,88,62,17,169,214,63,0,0,0,0,0,0,0,0,196,250,138,190,214,162,219,63,0,0,0,0,0,0,0,0,145,96,100,190,3,150,226,63,0,0,0,0,0,0,0,0,28,99,106,190,126,140,232,63,0,0,0,0,0,0,0,0,82,81,192,189,44,207,62,63,0,0,0,0,44,76,155,63,91,188,14,62,122,146,97,63,0,0,0,0,44,76,155,63,166,101,112,61,85,237,111,63,0,0,0,0,44,76,155,63,13,80,189,189,36,78,123,63,0,0,0,0,44,76,155,63,232,55,132,189,188,245,131,63,0,0,0,0,44,76,155,63,124,91,88,62,17,169,214,63,0,0,0,0,44,76,155,63,196,250,138,190,214,162,219,63,0,0,0,0,44,76,155,63,145,96,100,190,3,150,226,63,0,0,0,0,44,76,155,63,242,125,146,190,32,130,231,63,0,0,0,0,44,76,155,63,254,60,144,189,124,120,63,63,0,0,0,0,44,76,155,191,91,188,14,62,122,146,97,63,0,0,0,0,44,76,155,191,60,76,180,61,155,86,111,63,0,0,0,0,44,76,155,191,13,80,189,189,36,78,123,63,0,0,0,0,44,76,155,191,232,55,132,189,188,245,131,63,0,0,0,0,44,76,155,191,124,91,88,62,17,169,214,63,0,0,0,0,44,76,155,191,196,250,138,190,214,162,219,63,0,0,0,0,44,76,155,191,109,72,43,190,20,95,227,63,0,0,0,0,44,76,155,191,242,125,146,190,32,130,231,63,0,0,0,0,44,76,155,191,7,0,0,0,108,0,0,0,177,0,214,3,28,1,158,3,213,0,143,3,220,0,133,3,151,0,128,3,181,0,225,2,179,0,211,2,195,0,193,2,195,0,182,2,177,0,158,3,28,1,102,3,213,0,87,3,219,0,77,3,151,0,72,3,174,0,177,2,170,0,163,2,187,0,144,2,185,0,133,2,178,0,158,3,28,1,102,3,213,0,87,3,219,0,77,3,151,0,72,3,180,0,171,2,179,0,157,2,192,0,142,2,192,0,129,2,2,0,0,0,0,2,0,0,130,18,189,20,129,21,250,20,86,19,58,18,109,17,59,18,151,18,65,18,180,17,8,18,17,18,245,17,165,18,65,18,86,19,41,20,116,20,47,20,192,18,177,18,118,17,242,16,150,16,244,15,214,15,157,14,239,14,59,15,87,15,12,15,134,14,162,14,57,15,7,15,81,15,12,15,7,15,212,14,223,13,59,13,151,13,105,13,117,12,202,11,40,11,87,11,80,11,113,11,61,11,232,11,43,12,214,11,69,11,205,10,126,12,99,12,64,12,151,12,194,11,39,12,76,12,38,13,83,13,122,12,148,12,5,12,160,11,52,11,198,12,216,12,243,11,29,12,14,11,242,10,117,10,115,10,82,10,54,10,197,10,71,10,122,10,72,10,117,10,138,10,185,10,83,11,140,11,214,11,213,11,32,12,198,11,110,11,67,11,179,11,220,11,153,11,112,11,81,11,110,10,205,10,239,10,196,10,6,11,255,10,138,10,22,10,175,10,19,10,122,9,17,10,129,10,87,10,41,10,56,10,243,9,108,10,118,10,62,10,225,9,230,9,45,10,63,10,55,10,162,9,199,9,228,9,60,10,197,10,123,10,53,10,93,10,50,10,237,9,200,9,226,9,108,10,146,10,44,10,38,10,50,10,198,9,33,10,20,10,218,9,244,9,53,10,192,9,48,9,248,8,158,9,95,9,7,9,135,9,117,9,30,10,54,10,189,9,86,9,108,9,149,9,134,9,159,9,100,9,216,9,84,10,209,9,160,9,140,9,40,10,25,10,249,9,233,8,148,9,114,9,113,9,5,10,205,9,77,9,84,9,36,10,71,10,47,10,213,9,177,9,252,9,161,9,165,9,30,10,8,10,98,9,116,9,74,9,76,9,72,9,197,9,253,9,168,9,231,9,78,9,165,9,212,9,29,9,183,9,57,10,156,9,94,9,207,9,189,9,40,10,67,10,220,9,90,9,118,9,96,9,164,9,115,9,119,9,127,9,2,9,129,9,131,9,41,9,129,9,46,9,92,9,71,9,17,9,151,9,212,9,242,9,204,9,19,10,236,9,120,9,255,8,28,10,124,10,189,10,86,10,176,9,205,9,198,9,142,10,125,10,188,10,34,11,196,11,186,11,30,11,7,12,54,13,48,15,188,15,77,15,3,0,0,0,0,2,0,0,37,9,123,9,158,9,122,9,233,9,14,10,21,10,184,9,242,8,69,9,208,9,186,9,45,9,179,9,169,9,99,9,141,9,222,9,189,9,156,9,197,8,186,8,132,9,143,9,46,9,22,9,92,9,192,8,166,9,168,9,91,9,181,9,165,9,118,9,206,9,93,9,153,9,178,9,19,10,230,9,101,9,133,9,37,10,50,10,3,10,168,9,113,9,114,9,21,9,175,9,180,9,153,9,4,9,86,9,151,9,237,8,84,9,6,9,42,9,59,9,70,9,69,9,120,9,236,9,192,9,177,9,107,9,62,9,178,9,114,9,153,9,78,9,129,9,101,9,127,9,112,9,124,9,79,9,25,9,92,9,142,9,125,9,38,9,118,9,184,9,21,10,16,10,136,9,160,9,248,8,168,9,172,9,214,8,63,9,101,9,118,9,122,9,109,9,155,9,149,9,96,9,178,9,100,9,111,9,69,9,217,8,128,9,165,9,71,9,214,8,5,9,46,9,139,9,250,9,218,9,46,10,31,10,246,9,10,10,242,8,102,9,80,9,167,9,1,10,188,9,59,9,194,9,254,9,240,9,30,10,226,9,208,9,182,9,82,9,148,9,81,9,248,8,157,9,210,9,121,9,218,8,159,9,151,9,237,8,193,8,181,9,205,9,108,9,184,9,41,9,68,9,25,9,118,9,134,9,188,9,171,9,56,9,55,9,75,9,182,9,215,9,107,9,137,9,81,9,137,9,176,9,253,9,213,9,180,9,219,9,9,10,206,9,38,9,61,9,110,9,122,9,61,9,92,9,106,9,64,9,137,9,186,9,148,9,88,9,138,9,140,9,170,9,118,9,65,9,61,9,121,9,67,10,33,10,70,10,193,9,28,9,93,9,84,9,199,9,112,9,251,8,226,9,212,9,80,9,65,9,215,9,126,9,189,9,217,9,252,9,192,9,132,9,228,9,253,9,161,9,90,9,69,9,176,9,177,9,231,9,4,10,221,9,232,9,77,9,98,9,107,9,55,9,68,9,85,9,141,9,100,9,62,9,69,9,10,9,87,9,143,9,138,9,41,9,112,9,54,9,123,9,213,8,46,8,177,8,40,8,6,8,21,8,127,7,142,8,215,8,208,8,239,8,168,8,62,9,45,9,143,9,6,0,0,0,24,0,0,0,11,8,0,0,201,71,0,0,218,118,1,0,0,0,0,0,0,0,0,0,20,0,0,0,9,0,0,0,28,0,0,0,0,0,0,0,87,238,0,0,43,0,43,0,43,0,44,0,43,0,44,0,45,0,45,0,40,0,40,0,160,81,0,8,70,25,2,0,209,56,0,0,27,0,0,0,152,97,0,8,0,0,0,0};

uint16_t dataLen = 3459;
uint8_t raw_data[] = {1,1,1,2,1,4,3,6,5,8,7,4,0,5,3,192,6,0,0,67,104,10,0,21,0,0,0,95,100,108,215,27,0,0,0,6,0,0,0,0,0,0,0,1,0,0,0,176,1,0,0,82,81,192,189,44,207,62,63,0,0,0,0,0,0,0,0,91,188,14,62,122,146,97,63,0,0,0,0,0,0,0,0,60,76,180,61,155,86,111,63,0,0,0,0,0,0,0,0,13,80,189,189,36,78,123,63,0,0,0,0,0,0,0,0,232,55,132,189,188,245,131,63,0,0,0,0,0,0,0,0,124,91,88,62,17,169,214,63,0,0,0,0,0,0,0,0,196,250,138,190,214,162,219,63,0,0,0,0,0,0,0,0,145,96,100,190,3,150,226,63,0,0,0,0,0,0,0,0,28,99,106,190,126,140,232,63,0,0,0,0,0,0,0,0,82,81,192,189,44,207,62,63,0,0,0,0,44,76,155,63,91,188,14,62,122,146,97,63,0,0,0,0,44,76,155,63,166,101,112,61,85,237,111,63,0,0,0,0,44,76,155,63,13,80,189,189,36,78,123,63,0,0,0,0,44,76,155,63,232,55,132,189,188,245,131,63,0,0,0,0,44,76,155,63,124,91,88,62,17,169,214,63,0,0,0,0,44,76,155,63,196,250,138,190,214,162,219,63,0,0,0,0,44,76,155,63,145,96,100,190,3,150,226,63,0,0,0,0,44,76,155,63,242,125,146,190,32,130,231,63,0,0,0,0,44,76,155,63,254,60,144,189,124,120,63,63,0,0,0,0,44,76,155,191,91,188,14,62,122,146,97,63,0,0,0,0,44,76,155,191,60,76,180,61,155,86,111,63,0,0,0,0,44,76,155,191,13,80,189,189,36,78,123,63,0,0,0,0,44,76,155,191,232,55,132,189,188,245,131,63,0,0,0,0,44,76,155,191,124,91,88,62,17,169,214,63,0,0,0,0,44,76,155,191,196,250,138,190,214,162,219,63,0,0,0,0,44,76,155,191,109,72,43,190,20,95,227,63,0,0,0,0,44,76,155,191,242,125,146,190,32,130,231,63,0,0,0,0,44,76,155,191,7,0,0,0,108,0,0,0,177,0,214,3,28,1,158,3,213,0,143,3,220,0,133,3,151,0,128,3,181,0,225,2,179,0,211,2,195,0,193,2,195,0,182,2,177,0,158,3,28,1,102,3,213,0,87,3,219,0,77,3,151,0,72,3,174,0,177,2,170,0,163,2,187,0,144,2,185,0,133,2,178,0,158,3,28,1,102,3,213,0,87,3,219,0,77,3,151,0,72,3,180,0,171,2,179,0,157,2,192,0,142,2,192,0,129,2,2,0,0,0,0,2,0,0,130,18,189,20,129,21,250,20,86,19,58,18,109,17,59,18,151,18,65,18,180,17,8,18,17,18,245,17,165,18,65,18,86,19,41,20,116,20,47,20,192,18,177,18,118,17,242,16,150,16,244,15,214,15,157,14,239,14,59,15,87,15,12,15,134,14,162,14,57,15,7,15,81,15,12,15,7,15,212,14,223,13,59,13,151,13,105,13,117,12,202,11,40,11,87,11,80,11,113,11,61,11,232,11,43,12,214,11,69,11,205,10,126,12,99,12,64,12,151,12,194,11,39,12,76,12,38,13,83,13,122,12,148,12,5,12,160,11,52,11,198,12,216,12,243,11,29,12,14,11,242,10,117,10,115,10,82,10,54,10,197,10,71,10,122,10,72,10,117,10,138,10,185,10,83,11,140,11,214,11,213,11,32,12,198,11,110,11,67,11,179,11,220,11,153,11,112,11,81,11,110,10,205,10,239,10,196,10,6,11,255,10,138,10,22,10,175,10,19,10,122,9,17,10,129,10,87,10,41,10,56,10,243,9,108,10,118,10,62,10,225,9,230,9,45,10,63,10,55,10,162,9,199,9,228,9,60,10,197,10,123,10,53,10,93,10,50,10,237,9,200,9,226,9,108,10,146,10,44,10,38,10,50,10,198,9,33,10,20,10,218,9,244,9,53,10,192,9,48,9,248,8,158,9,95,9,7,9,135,9,117,9,30,10,54,10,189,9,86,9,108,9,149,9,134,9,159,9,100,9,216,9,84,10,209,9,160,9,140,9,40,10,25,10,249,9,233,8,148,9,114,9,113,9,5,10,205,9,77,9,84,9,36,10,71,10,47,10,213,9,177,9,252,9,161,9,165,9,30,10,8,10,98,9,116,9,74,9,76,9,72,9,197,9,253,9,168,9,231,9,78,9,165,9,212,9,29,9,183,9,57,10,156,9,94,9,207,9,189,9,40,10,67,10,220,9,90,9,118,9,96,9,164,9,115,9,119,9,127,9,2,9,129,9,131,9,41,9,129,9,46,9,92,9,71,9,17,9,151,9,212,9,242,9,204,9,19,10,236,9,120,9,255,8,28,10,124,10,189,10,86,10,176,9,205,9,198,9,142,10,125,10,188,10,34,11,196,11,186,11,30,11,7,12,54,13,48,15,188,15,77,15,3,0,0,0,0,2,0,0,37,9,123,9,158,9,122,9,233,9,14,10,21,10,184,9,242,8,69,9,208,9,186,9,45,9,179,9,169,9,99,9,141,9,222,9,189,9,156,9,197,8,186,8,132,9,143,9,46,9,22,9,92,9,192,8,166,9,168,9,91,9,181,9,165,9,118,9,206,9,93,9,153,9,178,9,19,10,230,9,101,9,133,9,37,10,50,10,3,10,168,9,113,9,114,9,21,9,175,9,180,9,153,9,4,9,86,9,151,9,237,8,84,9,6,9,42,9,59,9,70,9,69,9,120,9,236,9,192,9,177,9,107,9,62,9,178,9,114,9,153,9,78,9,129,9,101,9,127,9,112,9,124,9,79,9,25,9,92,9,142,9,125,9,38,9,118,9,184,9,21,10,16,10,136,9,160,9,248,8,168,9,172,9,214,8,63,9,101,9,118,9,122,9,109,9,155,9,149,9,96,9,178,9,100,9,111,9,69,9,217,8,128,9,165,9,71,9,214,8,5,9,46,9,139,9,250,9,218,9,46,10,31,10,246,9,10,10,242,8,102,9,80,9,167,9,1,10,188,9,59,9,194,9,254,9,240,9,30,10,226,9,208,9,182,9,82,9,148,9,81,9,248,8,157,9,210,9,121,9,218,8,159,9,151,9,237,8,193,8,181,9,205,9,108,9,184,9,41,9,68,9,25,9,118,9,134,9,188,9,171,9,56,9,55,9,75,9,182,9,215,9,107,9,137,9,81,9,137,9,176,9,253,9,213,9,180,9,219,9,9,10,206,9,38,9,61,9,110,9,122,9,61,9,92,9,106,9,64,9,137,9,186,9,148,9,88,9,138,9,140,9,170,9,118,9,65,9,61,9,121,9,67,10,33,10,70,10,193,9,28,9,93,9,84,9,199,9,112,9,251,8,226,9,212,9,80,9,65,9,215,9,126,9,189,9,217,9,252,9,192,9,132,9,228,9,253,9,161,9,90,9,69,9,176,9,177,9,231,9,4,10,221,9,232,9,77,9,98,9,107,9,55,9,68,9,85,9,141,9,100,9,62,9,69,9,10,9,87,9,143,9,138,9,41,9,112,9,54,9,123,9,213,8,46,8,177,8,40,8,6,8,21,8,127,7,142,8,215,8,208,8,239,8,168,8,62,9,45,9,143,9,6,0,0,0,24,0,0,0,11,8,0,0,201,71,0,0,218,118,1,0,0,0,0,0,0,0,0,0,20,0,0,0,9,0,0,0,28,0,0,0,0,0,0,0,87,238,0,0,43,0,43,0,43,0,44,0,43,0,44,0,45,0,45,0,40,0,40,0,160,81,0,8,70,25,2,0,209,56,0,0,27,0,0,0,152,97,0,8,0,0,0,0,2,1,4,3,6,5,8,7,4,0,5,3,192,6,0,0,67,104,10,0,21,0,0,0,95,100,108,215,27,0,0,0,6,0,0,0,0,0,0,0,1,0,0,0,176,1,0,0,82,81,192,189,44,207,62,63,0,0,0,0,0,0,0,0,91,188,14,62,122,146,97,63,0,0,0,0,0,0,0,0,60,76,180,61,155,86,111,63,0,0,0,0,0,0,0,0,13,80,189,189,36,78,123,63,0,0,0,0,0,0,0,0,232,55,132,189,188,245,131,63,0,0,0,0,0,0,0,0,124,91,88,62,17,169,214,63,0,0,0,0,0,0,0,0,196,250,138,190,214,162,219,63,0,0,0,0,0,0,0,0,145,96,100,190,3,150,226,63,0,0,0,0,0,0,0,0,28,99,106,190,126,140,232,63,0,0,0,0,0,0,0,0,82,81,192,189,44,207,62,63,0,0,0,0,44,76,155,63,91,188,14,62,122,146,97,63,0,0,0,0,44,76,155,63,166,101,112,61,85,237,111,63,0,0,0,0,44,76,155,63,13,80,189,189,36,78,123,63,0,0,0,0,44,76,155,63,232,55,132,189,188,245,131,63,0,0,0,0,44,76,155,63,124,91,88,62,17,169,214,63,0,0,0,0,44,76,155,63,196,250,138,190,214,162,219,63,0,0,0,0,44,76,155,63,145,96,100,190,3,150,226,63,0,0,0,0,44,76,155,63,242,125,146,190,32,130,231,63,0,0,0,0,44,76,155,63,254,60,144,189,124,120,63,63,0,0,0,0,44,76,155,191,91,188,14,62,122,146,97,63,0,0,0,0,44,76,155,191,60,76,180,61,155,86,111,63,0,0,0,0,44,76,155,191,13,80,189,189,36,78,123,63,0,0,0,0,44,76,155,191,232,55,132,189,188,245,131,63,0,0,0,0,44,76,155,191,124,91,88,62,17,169,214,63,0,0,0,0,44,76,155,191,196,250,138,190,214,162,219,63,0,0,0,0,44,76,155,191,109,72,43,190,20,95,227,63,0,0,0,0,44,76,155,191,242,125,146,190,32,130,231,63,0,0,0,0,44,76,155,191,7,0,0,0,108,0,0,0,177,0,214,3,28,1,158,3,213,0,143,3,220,0,133,3,151,0,128,3,181,0,225,2,179,0,211,2,195,0,193,2,195,0,182,2,177,0,158,3,28,1,102,3,213,0,87,3,219,0,77,3,151,0,72,3,174,0,177,2,170,0,163,2,187,0,144,2,185,0,133,2,178,0,158,3,28,1,102,3,213,0,87,3,219,0,77,3,151,0,72,3,180,0,171,2,179,0,157,2,192,0,142,2,192,0,129,2,2,0,0,0,0,2,0,0,130,18,189,20,129,21,250,20,86,19,58,18,109,17,59,18,151,18,65,18,180,17,8,18,17,18,245,17,165,18,65,18,86,19,41,20,116,20,47,20,192,18,177,18,118,17,242,16,150,16,244,15,214,15,157,14,239,14,59,15,87,15,12,15,134,14,162,14,57,15,7,15,81,15,12,15,7,15,212,14,223,13,59,13,151,13,105,13,117,12,202,11,40,11,87,11,80,11,113,11,61,11,232,11,43,12,214,11,69,11,205,10,126,12,99,12,64,12,151,12,194,11,39,12,76,12,38,13,83,13,122,12,148,12,5,12,160,11,52,11,198,12,216,12,243,11,29,12,14,11,242,10,117,10,115,10,82,10,54,10,197,10,71,10,122,10,72,10,117,10,138,10,185,10,83,11,140,11,214,11,213,11,32,12,198,11,110,11,67,11,179,11,220,11,153,11,112,11,81,11,110,10,205,10,239,10,196,10,6,11,255,10,138,10,22,10,175,10,19,10,122,9,17,10,129,10,87,10,41,10,56,10,243,9,108,10,118,10,62,10,225,9,230,9,45,10,63,10,55,10,162,9,199,9,228,9,60,10,197,10,123,10,53,10,93,10,50,10,237,9,200,9,226,9,108,10,146,10,44,10,38,10,50,10,198,9,33,10,20,10,218,9,244,9,53,10,192,9,48,9,248,8,158,9,95,9,7,9,135,9,117,9,30,10,54,10,189,9,86,9,108,9,149,9,134,9,159,9,100,9,216,9,84,10,209,9,160,9,140,9,40,10,25,10,249,9,233,8,148,9,114,9,113,9,5,10,205,9,77,9,84,9,36,10,71,10,47,10,213,9,177,9,252,9,161,9,165,9,30,10,8,10,98,9,116,9,74,9,76,9,72,9,197,9,253,9,168,9,231,9,78,9,165,9,212,9,29,9,183,9,57,10,156,9,94,9,207,9,189,9,40,10,67,10,220,9,90,9,118,9,96,9,164,9,115,9,119,9,127,9,2,9,129,9,131,9,41,9,129,9,46,9,92,9,71,9,17,9,151,9,212,9,242,9,204,9,19,10,236,9,120,9,255,8,28,10,124,10,189,10,86,10,176,9,205,9,198,9,142,10,125,10,188,10,34,11,196,11,186,11,30,11,7,12,54,13,48,15,188,15,77,15,3,0,0,0,0,2,0,0,37,9,123,9,158,9,122,9,233,9,14,10,21,10,184,9,242,8,69,9,208,9,186,9,45,9,179,9,169,9,99,9,141,9,222,9,189,9,156,9,197,8,186,8,132,9,143,9,46,9,22,9,92,9,192,8,166,9,168,9,91,9,181,9,165,9,118,9,206,9,93,9,153,9,178,9,19,10,230,9,101,9,133,9,37,10,50,10,3,10,168,9,113,9,114,9,21,9,175,9,180,9,153,9,4,9,86,9,151,9,237,8,84,9,6,9,42,9,59,9,70,9,69,9,120,9,236,9,192,9,177,9,107,9,62,9,178,9,114,9,153,9,78,9,129,9,101,9,127,9,112,9,124,9,79,9,25,9,92,9,142,9,125,9,38,9,118,9,184,9,21,10,16,10,136,9,160,9,248,8,168,9,172,9,214,8,63,9,101,9,118,9,122,9,109,9,155,9,149,9,96,9,178,9,100,9,111,9,69,9,217,8,128,9,165,9,71,9,214,8,5,9,46,9,139,9,250,9,218,9,46,10,31,10,246,9,10,10,242,8,102,9,80,9,167,9,1,10,188,9,59,9,194,9,254,9,240,9,30,10,226,9,208,9,182,9,82,9,148,9,81,9,248,8,157,9,210,9,121,9,218,8,159,9,151,9,237,8,193,8,181,9,205,9,108,9,184,9,41,9,68,9,25,9,118,9,134,9,188,9,171,9,56,9,55,9,75,9,182,9,215,9,107,9,137,9,81,9,137,9,176,9,253,9,213,9,180,9,219,9,9,10,206,9,38,9,61,9,110,9,122,9,61,9,92,9,106,9,64,9,137,9,186,9,148,9,88,9,138,9,140,9,170,9,118,9,65,9,61,9,121,9,67,10,33,10,70,10,193,9,28,9,93,9,84,9,199,9,112,9,251,8,226,9,212,9,80,9,65,9,215,9,126,9,189,9,217,9,252,9,192,9,132,9,228,9,253,9,161,9,90,9,69,9,176,9,177,9,231,9,4,10,221,9,232,9,77,9,98,9,107,9,55,9,68,9,85,9,141,9,100,9,62,9,69,9,10,9,87,9,143,9,138,9,41,9,112,9,54,9,123,9,213,8,46,8,177,8,40,8,6,8,21,8,127,7,142,8,215,8,208,8,239,8,168,8,62,9,45,9,143,9,6,0,0,0,24,0,0,0,11,8,0,0,201,71,0,0,218,118,1,0,0,0,0,0,0,0,0,0,20,0,0,0,9,0,0,0,28,0,0,0,0,0,0,0,87,238,0,0,43,0,43,0,43,0,44,0,43,0,44,0,45,0,45,0,40,0,40,0,160,81,0,8,70,25,2,0,209,56,0,0,27,0,0,0,152,97,0,8,0,0,0,0};
uint8_t magicWord[] = {2,1,4,3,6,5,8,7};
uint16_t numframesAvailable = 0;
uint32_t totalPacketLen = 0;
#define MMWDEMO_UART_MSG_DETECTED_POINTS			1
#define MMWDEMO_UART_MSG_RANGE_PROFILE				2
#define MMWDEMO_UART_MSG_NOISE_PROFILE				3
#define MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO	7

struct structHeader {
	uint16_t magicWord[8];
	uint32_t version[4];
	uint32_t totalPacketLen;
    uint32_t platform;
	uint32_t frameNumber;
	uint32_t timeCpuCycles;
	uint32_t numDetectedObj;
	uint32_t numTLVs;
	uint32_t subFrameNumber;
	uint32_t numStaticDetectedObj;
	uint32_t idX;
};

struct structTLV {
	uint32_t type;
    uint32_t length;
	vector<uint8_t> payload;
};

struct pointStruct
{
	vector<float> x;
	vector<float> y ;
	vector<float> z ;
	vector<float> doppler ;
} ptCloud;

union byte2float
{
	int str[4];
	vector<uint8_t>  myByte;
	vector<float> myFloat;
	~byte2float() {}
};

void Swap(float &a, float &b)
{
    float temp = a;
    a = b;
    b = temp;
}

void BubbleSort(vector<float> a, uint32_t n)
{	
	for (int i = 0; i < n - 1; i++)
		for (int j = n - 1; j > i; j--)
		   if(a[j] < a[j-1])
		       Swap(a[j], a[j-1]);
}

structHeader getFrameHeader (uint8_t framePacket[], uint16_t dataLen)
{
	structHeader frameHeader;
	// check that all packet has been read
    frameHeader.totalPacketLen = framePacket[12] + framePacket[13] * 256.0 + framePacket[14] * 65536.0 + framePacket[15] * 1.6777216E+7;
	uint32_t idX = 0;
	// Read the header
	if ((dataLen >= frameHeader.totalPacketLen) && (dataLen != 0))
	{
		//word array to convert 4 bytes to a 32 bit number
        //word = [1, 2**8, 2**16, 2**24]

        // Initialize the pointer index
        for (auto idX = 0; idX < 8; idX++)
		{
			frameHeader.magicWord[idX] = framePacket[idX];
		}
		idX += 8;
		 for (auto idX = 0; idX < 4; idX++)
		{
			frameHeader.version[idX] = framePacket[idX + 8];
		}
		idX += 4;
        frameHeader.totalPacketLen = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.platform = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.frameNumber = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.timeCpuCycles = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.numDetectedObj = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.numTLVs = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
		frameHeader.subFrameNumber = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
 	}

	frameHeader.idX = idX;

	return frameHeader;
}

structTLV getTLV (uint8_t framePacket[], uint32_t numTLVs, uint16_t idX1)
{
	structTLV tlv;

	// Check the header of the TLV message
	for (auto tlvIdx = 0; tlvIdx < numTLVs; tlvIdx++)
	{
		tlv.type = framePacket[idX1]*1 + framePacket[idX1 + 1]*256.0 + framePacket[idX1 + 2]*65536.0 + framePacket[idX1 + 3]*1.6777216E+7;
		idX1 += 4;
		tlv.length = framePacket[idX1]*1 + framePacket[idX1 + 1]*256.0 + framePacket[idX1 + 2]*65536.0 + framePacket[idX1 + 3]*1.6777216E+7;
		idX1 += 4;
		for (auto i = 0; i < tlv.length ; i++)
			{
				tlv.payload.push_back(framePacket[idX1 + i]);
			}
		idX1 += tlv.length;

		switch (tlv.type)
		{
			case MMWDEMO_UART_MSG_DETECTED_POINTS :
			{
				// getGtrackPtCloud(payload)
				uint32_t length = tlv.length;
				int numDetectedObj = length/16;
				byte2float data = {0};

				if (numDetectedObj)
				{
					for (auto i = 0; i < length; i++)
						{
							data.myByte.push_back(tlv.payload[i]);
						}

					for (auto i = 0; i < numDetectedObj; i++)
					{
						ptCloud.x.push_back(data.myFloat[i * 4]);
						ptCloud.y.push_back(data.myFloat[i * 4 + 1]);
						ptCloud.z.push_back(data.myFloat[i * 4 + 2]);
						ptCloud.doppler.push_back(data.myFloat[i * 4 + 3]);
					}
					int a = 5;
				}
			}
			break;

			case MMWDEMO_UART_MSG_RANGE_PROFILE:
			break;
			case MMWDEMO_UART_MSG_NOISE_PROFILE:
			break;
			case MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO:
			break;
			default:
			break;
		}

		
	}
	return tlv1;
}

int main()
{
	// Check for all possible locations of the magic word
	for (uint16_t i = 0; i < dataLen - 7; i++)
	{
		if (raw_data[i] == 2 && raw_data[i+1] == 1 && raw_data[i+2] == 4 && raw_data[i+3] == 3 && raw_data[i+4] == 6 && raw_data[i+5] == 5 && raw_data[i+6] == 8 && raw_data[i+7] == 7)
		{
			numframesAvailable++;
		}
	}

	uint32_t startIdx[numframesAvailable + 1];
	uint16_t framesAvailable = 0;

	for (uint16_t i = 0; i < dataLen - 7; i++)
	{
		if (raw_data[i] == 2 && raw_data[i+1] == 1 && raw_data[i+2] == 4 && raw_data[i+3] == 3 && raw_data[i+4] == 6 && raw_data[i+5] == 5 && raw_data[i+6] == 8 && raw_data[i+7] == 7)
		{
			startIdx[framesAvailable] = i;
			framesAvailable++;
		}
	}

	// Check that startIdx is not empty
	startIdx[numframesAvailable] = dataLen;
	uint8_t bytesBuffer[(startIdx[1] - startIdx[0])];
    if (numframesAvailable)
	{
		//Remove the data before the first start index
		if (startIdx[0] > 0 && startIdx[0] < dataLen)
		{
			for (auto i = 0; i < (startIdx[1] - startIdx[0]); i++)
			{
				bytesBuffer[i] = raw_data[startIdx[0] + i];
			}

            //update dataLen
			dataLen = sizeof bytesBuffer / sizeof(uint8_t);
		}
	}

	// Read the total packet length // as only 1 frame in buffer
	uint8_t framePacket[dataLen];
	for (auto i = 0; i < dataLen; i++)
	{
		framePacket[i] = bytesBuffer[i];
	}

	structHeader frameHeader = getFrameHeader(framePacket, dataLen);
	uint32_t idX = frameHeader.idX;


	// Read the TLV messages
	structTLV tlv = getTLV(framePacket, frameHeader.numTLVs, idX);





	sort(ptCloud.y.begin(), ptCloud.y.end());
	

	
	std::cout << "kq" << ptCloud.y[0];

	// Xuất kết quả nghiệm x
	std::cout << "x = " << (6 / 2) << std::endl;

	return 0;

}