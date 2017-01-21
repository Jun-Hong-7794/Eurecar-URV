#ifndef __Constants__
#define __Constants__

/******************** SetCommand Device ********************/
#define UGV_DEF_G  0

#define UGV_DEF_M  1
#define UGV_DEF_P  2
#define UGV_DEF_S  3
#define UGV_DEF_C  4
#define UGV_DEF_CB  5
#define UGV_DEF_VAR  6

#define UGV_DEF_AC  7
#define UGV_DEF_DC  8

#define UGV_DEF_DS  9
#define UGV_DEF_D1  10
#define UGV_DEF_D0  11


#define UGV_DEF_H  13
#define UGV_DEF_EX  14
#define UGV_DEF_MG  15
#define UGV_DEF_MS  16

#define UGV_DEF_PR  17
#define UGV_DEF_PX  18
#define UGV_DEF_PRX  19
#define UGV_DEF_AX  20
#define UGV_DEF_DX  21
#define UGV_DEF_B  22
#define UGV_DEF_SX  23

#define UGV_DEF_CS  24


#define UGV_DEF_RC  26

#define UGV_DEF_EES  27
#define UGV_DEF_BND  28


/******************** SetCommand Alias ********************/
#define UGV_DEF_GO  0

#define UGV_DEF_MOTCMD  1
#define UGV_DEF_MOTPOS  2
#define UGV_DEF_MOTVEL  3
#define UGV_DEF_SENCNTR  4
#define UGV_DEF_SBLCNTR  5
#define UGV_DEF_VAR  6

#define UGV_DEF_ACCEL  7
#define UGV_DEF_DECEL  8

#define UGV_DEF_DOUT  9
#define UGV_DEF_DSET  10
#define UGV_DEF_DRES  11


#define UGV_DEF_HOME  13
#define UGV_DEF_ESTOP  14
#define UGV_DEF_MGO  15
#define UGV_DEF_MSTOP  16

#define UGV_DEF_MPOSREL  17
#define UGV_DEF_NXTPOS  18
#define UGV_DEF_NXTPOSR  19
#define UGV_DEF_NXTACC  20
#define UGV_DEF_NXTDEC  21
#define UGV_DEF_BOOL  22
#define UGV_DEF_NXTVEL  23

#define UGV_DEF_CANSEND  24


#define UGV_DEF_RCOUT  26

#define UGV_DEF_EESAV  27
#define UGV_DEF_BIND  28


/******************** GetValue Device ********************/
#define UGV_DEF_A  0

#define UGV_DEF_M  1
#define UGV_DEF_P  2
#define UGV_DEF_S  3
#define UGV_DEF_C  4
#define UGV_DEF_CB  5
#define UGV_DEF_VAR  6

#define UGV_DEF_SR  7
#define UGV_DEF_CR  8
#define UGV_DEF_CBR  9
#define UGV_DEF_BS  10
#define UGV_DEF_BSR  11
#define UGV_DEF_BA  12
#define UGV_DEF_V  13
#define UGV_DEF_D  14
#define UGV_DEF_DI  15
#define UGV_DEF_AI  16
#define UGV_DEF_PI  17
#define UGV_DEF_T  18
#define UGV_DEF_F  19
#define UGV_DEF_FS  20
#define UGV_DEF_FF  21
#define UGV_DEF_B  22
#define UGV_DEF_DO  23
#define UGV_DEF_E  24

#define UGV_DEF_CIS  25
#define UGV_DEF_CIA  26
#define UGV_DEF_CIP  27

#define UGV_DEF_TM  28

#define UGV_DEF_LK  29




#define UGV_DEF_TR  32

#define UGV_DEF_K  33
#define UGV_DEF_DR  34

#define UGV_DEF_AIC  35
#define UGV_DEF_PIC  36

#define UGV_DEF_MA  37
#define UGV_DEF_CL  38
#define UGV_DEF_CAN  39
#define UGV_DEF_CF  40

#define UGV_DEF_MGD  41
#define UGV_DEF_MGT  42
#define UGV_DEF_MGM  43
#define UGV_DEF_MGS  44
#define UGV_DEF_MGY  45




#define UGV_DEF_FM  48
#define UGV_DEF_HS  49


/******************** GetValue Alias ********************/
#define UGV_DEF_MOTAMPS  0

#define UGV_DEF_MOTCMD  1
#define UGV_DEF_MOTPWR  2
#define UGV_DEF_ABSPEED  3
#define UGV_DEF_ABCNTR  4
#define UGV_DEF_BLCNTR  5
#define UGV_DEF_VAR  6

#define UGV_DEF_RELSPEED  7
#define UGV_DEF_RELCNTR  8
#define UGV_DEF_BLRCNTR  9
#define UGV_DEF_BLSPEED  10
#define UGV_DEF_BLRSPEED  11
#define UGV_DEF_BATAMPS  12
#define UGV_DEF_VOLTS  13
#define UGV_DEF_DIGIN  14
#define UGV_DEF_DIN  15
#define UGV_DEF_ANAIN  16
#define UGV_DEF_PLSIN  17
#define UGV_DEF_TEMP  18
#define UGV_DEF_FEEDBK  19
#define UGV_DEF_STFLAG  20
#define UGV_DEF_FLTFLAG  21
#define UGV_DEF_BOOL  22
#define UGV_DEF_DIGOUT  23
#define UGV_DEF_LPERR  24

#define UGV_DEF_CMDSER  25
#define UGV_DEF_CMDANA  26
#define UGV_DEF_CMDPLS  27

#define UGV_DEF_TIME  28

#define UGV_DEF_LOCKED  29




#define UGV_DEF_TRACK  32

#define UGV_DEF_SPEKTRUM  33
#define UGV_DEF_DREACHED  34

#define UGV_DEF_ANAINC  35
#define UGV_DEF_PLSINC  36

#define UGV_DEF_MEMS  37
#define UGV_DEF_CALIVE  38
#define UGV_DEF_CAN  39
#define UGV_DEF_CF  40

#define UGV_DEF_MGDET  41
#define UGV_DEF_MGTRACK  42
#define UGV_DEF_MGMRKR  43
#define UGV_DEF_MGSTATUS  44
#define UGV_DEF_MGYRO  45




#define UGV_DEF_MOTFLAG  48
#define UGV_DEF_HSENSE  49


/******************** SetConfig/GetConfig Alias ********************/
#define UGV_DEF_EE  0

#define UGV_DEF_BKD  1
#define UGV_DEF_OVL  2
#define UGV_DEF_UVL  3
#define UGV_DEF_THLD  4
#define UGV_DEF_MXMD  5
#define UGV_DEF_PWMF  6

#define UGV_DEF_CPRI  7
#define UGV_DEF_RWD  8
#define UGV_DEF_ECHOF  9
#define UGV_DEF_RSBR  10
#define UGV_DEF_ACS  11
#define UGV_DEF_AMS  12
#define UGV_DEF_CLIN  13
#define UGV_DEF_DFC  14

#define UGV_DEF_DINA  15
#define UGV_DEF_DINL  16
#define UGV_DEF_DOA  17
#define UGV_DEF_DOL  18

#define UGV_DEF_AMOD  19
#define UGV_DEF_AMIN  20
#define UGV_DEF_AMAX  21
#define UGV_DEF_ACTR  22
#define UGV_DEF_ADB  23
#define UGV_DEF_ALIN  24
#define UGV_DEF_AINA  25
#define UGV_DEF_AMINA  26
#define UGV_DEF_AMAXA  27
#define UGV_DEF_APOL  28

#define UGV_DEF_PMOD  29
#define UGV_DEF_PMIN  30
#define UGV_DEF_PMAX  31
#define UGV_DEF_PCTR  32
#define UGV_DEF_PDB  33
#define UGV_DEF_PLIN  34
#define UGV_DEF_PINA  35
#define UGV_DEF_PMINA  36
#define UGV_DEF_PMAXA  37
#define UGV_DEF_PPOL  38

#define UGV_DEF_MMOD  39
#define UGV_DEF_MXPF  40
#define UGV_DEF_MXPR  41
#define UGV_DEF_ALIM  42
#define UGV_DEF_ATRIG  43
#define UGV_DEF_ATGA  44
#define UGV_DEF_ATGD  45
#define UGV_DEF_KP  46
#define UGV_DEF_KI  47
#define UGV_DEF_KD  48
#define UGV_DEF_PIDM  49
#define UGV_DEF_ICAP  50
#define UGV_DEF_MAC  51
#define UGV_DEF_MDEC  52
#define UGV_DEF_MVEL  53
#define UGV_DEF_MXRPM  54
#define UGV_DEF_MXTRN  55
#define UGV_DEF_CLERD  56

#define UGV_DEF_BPOL  57
#define UGV_DEF_BLSTD  58
#define UGV_DEF_BLFB  59
#define UGV_DEF_BHOME  60
#define UGV_DEF_BLL  61
#define UGV_DEF_BHL  62
#define UGV_DEF_BLLA  63
#define UGV_DEF_BHLA  64

#define UGV_DEF_SXC  65
#define UGV_DEF_SXM  66








#define UGV_DEF_EMOD  72
#define UGV_DEF_EPPR  73
#define UGV_DEF_ELL  74
#define UGV_DEF_EHL  75
#define UGV_DEF_ELLA  76
#define UGV_DEF_EHLA  77
#define UGV_DEF_EHOME  78

#define UGV_DEF_SKUSE  79
#define UGV_DEF_SKMIN  80
#define UGV_DEF_SKMAX  81
#define UGV_DEF_SKCTR  82
#define UGV_DEF_SKDB  83
#define UGV_DEF_SKLIN  84

#define UGV_DEF_CEN  85
#define UGV_DEF_CNOD  86

#define UGV_DEF_CHB  88
#define UGV_DEF_CAS  89
#define UGV_DEF_CLSN  90
#define UGV_DEF_CSRT  91
#define UGV_DEF_CTPS  92

#define UGV_DEF_SCRO  93

#define UGV_DEF_BMOD  94
#define UGV_DEF_BADJ  95
#define UGV_DEF_BADV  96
#define UGV_DEF_BZPW  97

////////////////////////////////////////////////////UGV

#define UGV_move_forward 1
#define UGV_move_backward 2
#define UGV_move_left 3
#define UGV_move_right 4


#endif
