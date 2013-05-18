#define CMD_TURNON 0x01
#define CMD_ACTIVATED 0x02
#define CMD_READY 0x03
#define CMD_RC_CONFIRM 0x04
#define CMD_RC_PROMPT 0x05
#define CMD_MODE_PROMPT 0x06
#define CMD_IDLE_PROMPT 0x0B // 0x0C,0x0D,0x0E all the same
#define CMD_HUMMING_PROMPT 0x0F
#define CMD_COUGH_PROMPT 0x10
#define CMD_TIRED_PROMPT 0x11
#define CMD_SLEEP_PROMPT 0x12
#define CMD_FART 0x40 // 2A
#define CMD_ALLYOURBASES 0x41
#define CMD_SHOOT_RIGHT 0x64
#define CMD_SHOOT_RIGHT2 0x68
#define CMD_SHOOT2 0x69
#define CMD_BEEP 0x6a
#define CMD_BANZAI 0x7F
#define CMD_CHEER1 0x90
#define CMD_CHEER2 0x91
#define CMD_DOG 0x92
#define CMD_CAR 0x93
#define CMD_EAGLE 0x94
#define CMD_ROOSTER 0x95
#define CMD_GORILLA 0x96
#define CMD_LOOKOUT 0xA1
#define CMD_STORY1 0xA2 // knight and princess
#define CMD_STORY2 0xA3 // ready to start day
#define CMD_GREET1 0xA4 // good morning
#define CMD_GREET2 0xA5 // do somthing fun
#define CMD_POOP 0xA6 // poops his pants
#define CMD_GOOUT 0xA7 // ready to go out dancing
#define CMD_HIBUDDY 0xA8 // .. bring a round of drinks
#define CMD_INTRODUCTION 0xA9
#define CMD_ATYOURSERVICE 0xAA
#define CMD_SMELLS 0xAB
#define CMD_THATWASCLOSE 0xAC
#define CMD_WANNAPICEOFME 0xAD
#define CMD_RUNFORYOURLIFE 0xAE
#define CMD_TONEWTODIE 0xAF
// 0xB0 - nothing?
#define CMD_SWANLAKE 0xB1
#define CMD_DISCO 0xB2
#define CMD_MOONWALK 0xB3
#define CMD_REPEAT_PROMPT 0xB4
#define CMD_REPEAT_PROMPT2 0xB5
#define CMD_REPEAT_PROMPT3 0xB6
// 0xB7-0xC4 steps in different directions
#define CMD_HEADSMASH 0xC5
#define CMD_HEADHIT 0xC6
// 0xCC-0xD2 - unknown (use param?)
#define CMD_HIBEEP 0xD3 
// 0xD4 - unknown (use param?)
#define CMD_BEND_BACK 0xD8 // same up to 0xDB
#define CMD_SQUAT 0xDB // also 0xDC
#define CMD_BEND_FORWARD 0xDD
#define CMD_HEAD_LEFT_60 0xDE
#define CMD_HEAD_LEFT_45 0xDF
#define CMD_HEAD_LEFT_30 0xE0
#define CMD_HEAD_RIGHT_30 0xE1
#define CMD_HEAD_RIGHT_45 0xE2
#define CMD_HEAD_RIGHT_60 0xE3
// seems identical to A & B getups
#define CMD_GETUP_BELLY 0xE4
#define CMD_GETUP_BACK 0xE5
// E6 unknown 
#define CMD_HEAD_SCAN_AND_BEND 0xE7
#define CMD_ARM_TEST 0xE8
#define CMD_FALL_AND_LEG_TEST 0xE9
#define CMD_THANKYOUSIR 0xEA
#define CMD_ILOVEYOU_SHORT 0xEB
// CMD_3BEEPS goes through some test sequence and also turns sounds (starts?) 
// into beeps for all commands (power off to quit this mode)
// (looks like a tool to synchronize sound with moves)
#define CMD_3BEEPS 0xEC
#define CMD_FALL_DEAD 0xED
#define CMD_3BEEPS_AND_SLIDE 0xEE
// EF-FF unknown










#define CMD_RC	0x07
#define CMD_PM	0x08
#define CMD_SA	0x09
#define CMD_VC	0x0a
#define CMD_1P	0x13
#define CMD_2P	0x14
#define CMD_3P	0x15
#define CMD_4P	0x16
#define CMD_11P	0x17
#define CMD_12P	0x18
#define CMD_13P	0x19
#define CMD_14P	0x1a
#define CMD_21P	0x1b
#define CMD_22P	0x1c
#define CMD_23P	0x1d
#define CMD_24P	0x1e
#define CMD_31P	0x1f
#define CMD_32P	0x20
#define CMD_34P	0x21
#define CMD_1K	0x22
#define CMD_2K	0x23
#define CMD_3K	0x24
#define CMD_4K	0x25
#define CMD_11K	0x26
#define CMD_12K	0x27
#define CMD_13K	0x28
#define CMD_14K	0x29
#define CMD_31K	0x2a
#define CMD_42K	0x2b
#define CMD_21K	0x2c
#define CMD_22K	0x2d
#define CMD_23K	0x2e
#define CMD_24K	0x2f
#define CMD_31K	0x30
#define CMD_34K	0x31
#define CMD_3G	0x32
#define CMD_2G	0x33
#define CMD_3G	0x34
#define CMD_4G	0x35
#define CMD_11G	0x36
#define CMD_12G	0x37
#define CMD_13G	0x38
#define CMD_14G	0x39
#define CMD_21G	0x3a
#define CMD_22G	0x3b
#define CMD_23G	0x3c
#define CMD_A	0x3d
#define CMD_B	0x3e
#define CMD_1A	0x3f
#define CMD_2A	0x40
#define CMD_3A	0x41
#define CMD_4A	0x42
#define CMD_4A	0x43
#define CMD_11A	0x43
#define CMD_12A	0x44
#define CMD_13A	0x45
#define CMD_14A	0x46
#define CMD_21A	0x47
#define CMD_22A	0x48
#define CMD_23A	0x40
#define CMD_32A	0x4a
#define CMD_31A	0x4a
#define CMD_32A	0x4c
#define CMD_33A	0x4c
#define CMD_34A	0x4d
#define CMD_41A	0x4e
#define CMD_42A	0x4f
#define CMD_43A	0x50
#define CMD_111A	0x51
#define CMD_222A	0x52
#define CMD_333A	0x53
#define CMD_11B	0x54
#define CMD_12B	0x55
#define CMD_13B	0x56
#define CMD_14B	0x57
#define CMD_31B	0x58
#define CMD_22B	0x59
#define CMD_23B	0x5a
#define CMD_24B	0x5b
#define CMD_31B	0x5c
#define CMD_32B	0x5d
#define CMD_33B	0x5e
#define CMD_234B	0x5f
#define CMD_41B	0x60
#define CMD_42B	0x61
#define CMD_43B	0x62
#define CMD_44B	0x63
#define CMD_112A	0x65
#define CMD_113A	0x66
#define CMD_114A	0x67
#define CMD_124A	0x6b
#define CMD_131A	0x6c
#define CMD_132A	0x6d
#define CMD_113B	0x6e
#define CMD_114B	0x6f
#define CMD_121B	0x70
#define CMD_122B	0x71
#define CMD_123B	0x72
#define CMD_124B	0x73
#define CMD_131B	0x74
#define CMD_132B	0x75
#define CMD_133B	0x76
#define CMD_134B	0x77
#define CMD_141A	0x78
#define CMD_143A	0x79
#define CMD_143B	0x7a
#define CMD_144A	0x7b
#define CMD_211B	0x7c
#define CMD_212B	0x7d
#define CMD_213B	0x7e
#define CMD_221B	0x80
#define CMD_222B	0x81
#define CMD_223B	0x82
#define CMD_224B	0x83
#define CMD_232B	0x85
#define CMD_233B	0x86
#define CMD_241B	0x88
#define CMD_242B	0x89
#define CMD_A	0x8a
#define CMD_B	0x8b
#define CMD_AB	0x8c
#define CMD_AAA	0x8d
#define CMD_BBB	0x8e
#define CMD_BAB	0x8f
#define CMD_BBA	0x97
#define CMD_ABA	0x98
#define CMD_ABAB	0x99
#define CMD_AAAA	0x9a
#define CMD_FWRD	0xb7
#define CMD_BWRD	0xb8
#define CMD_FWLT	0xb9
#define CMD_FWRT	0xba
#define CMD_LEFT	0xbb
#define CMD_RGHT	0xbc
#define CMD_BKLT	0xbd
#define CMD_BKRT	0xbe
#define CMD_411A	0xc7
#define CMD_412A	0xc8
#define CMD_413A	0xc9
#define CMD_444B	0xca
#define CMD_444A	0xcb
#define CMD_LVSoff	0xd3
#define CMD_HP	0xd5
#define CMD_NOIMP	0xd6
#define CMD_END	0xd7
#define MSG_NOIMP	0x848080
#define MSG_NOIMP	0x848080
#define MSG_RUP	0x878280
#define MSG_RDW	0x808280
#define MSG_RRT	0x8480f0
#define MSG_RLT	0x848080
#define MSG_LUP	0x84f080
#define MSG_LDW	0x841080
#define MSG_LRT	0xec8080
#define MSG_LLT	0x0c8080
/* missing codes
0-6 00-06
11-11 0B-0B
13-18 0D-12
100-100 64-64
104-106 68-6A
122-122 7A-7A
127-127 7F-7F
144-148 90-94
150-150 96-96
155-182 9B-B6
191-198 BF-C6
204-210 CC-D2
212-212 D4-D4
216-235 D8-EB
*/