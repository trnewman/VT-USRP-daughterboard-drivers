#ifndef DB_RFIC_H
#define DB_RFIC_H
#include <db_rfic.h>
#include <db_base_impl.h>
#include <cmath>
#include <boost/thread.hpp>
#include <boost/weak_ptr.hpp>

int TX_EN = (1<<6);

struct rfic_key {
  std::string serial_no;
  int which;

  bool operator==(const rfic_key &x){
    return x.serial_no ==serial_no && x.which == which;
  }
};

class rfic{

	public:
	usrp_basic *d_raw_usrp;
	int d_which;
	bool d_is_shutdown;
  	int d_spi_format, d_spi_enable,d_spi_format_no_header;
	int Ngt3, NorNdiv4, RorFrNpRdiv4_25to18, RorFrNpRdiv4_17to10, RorFrNpRdiv4_9to2, RorFrNpRdiv4_1to0, Qu_tx_Ngt3 	\
	,NorNdiv4_phsh, RorFrNpRdiv4_phsh_25to18, RorFrNpRdiv4_phsh_17to10, RorFrNpRdiv4_phsh_9to2,			\
	RorFrNpRdiv4_phsh_1to0, Passthru_ref_clk, Byp_ram, Dis_adr_dith, Dis_p5G_dith, Byp_fine, Exclude32, Dis_risedge \
	, Dis_faledge, Spr_puls_en, Spr_puls_val_a_9to3, Spr_pulse_val_2to0, Spr_puls_val_b_9to2, Spr_puls_val_b_1to0	\
	,Thru_ris_en, Thru_ris_tap_11to6, Thru_ris_tap_5to0, Thru_fal_en, Thru_fal_tap_11to6, Thru_fal_tap_5to0, 	\
	Dig_delay, Clk_driver_en, qu_reg_en, qq_reg_en, win_rst, fineEn, fineEnb, rsffEn, dl_en, cp_en, forceCpUpb, 	\
	forceCpDn, pdUpTune_1to0, pdDnTune_1to0, cpUpTune_2to0, cpDnTune_2to0, pdEn, digClkPhase_7to0, Rst_n_async;
	
	std::string L1_lup00_15to8; //Read-only//
	std::string L1_lup90_15to8; //Read-only//
	std::string Merg_ris_fin;   //Read-only//
	std::string Merg_fal_fin;   //Read-only//
		
	int Qg00degDelay_0to4 ,Qg90degDelay_0to4, Qg180degDelay_0to4, Qg270degDelay_0to4, DischargeTap16_0to3,		\
	ChargeTap16_0to3, DischargeTapn_0to3, ChargeTapn16_0to3, X1sel_32to39, X1sel_40to47,X2sel_32to36, X2sel_37to41	\
	,X4sel_32to36, X4sel_37to41, X8sel_32to36, X8sel_41, X8sel_37to40, qutx_fwd_180Cal_en, qutx_fwd_0Cal_en;		
	//-------------------------------------------------------------------------------------------------------
	// TRANSMIT FEEDBACK QuIET FREQUENCY GENERATOR
	//-------------------------------------------------------------------------------------------------------
	int Ngt3_2, NorNdiv4_2, RorFrNpRdiv4_25to18_2, RorFrNpRdiv4_17to10_2, RorFrNpRdiv4_9to2_2, RorFrNpRdiv4_1to0_2,	\
	Qu_tx_Ngt3_2, NorNdiv4_phsh_2, RorFrNpRdiv4_phsh_25to18_2, RorFrNpRdiv4_phsh_17to10_2, RorFrNpRdiv4_phsh_9to2_2,	\
	RorFrNpRdiv4_phsh_1to0_2, Passthru_ref_clk_2, Byp_ram_2, Dis_adr_dith_2, Dis_p5G_dith_2, Byp_fine_2, 		\
	Exclude32_2, Dis_risedge_2, Dis_faledge_2, Spr_puls_en_2, Spr_puls_val_a_9to3_2, Spr_pulse_val_2to0_2, 		\
	Spr_puls_val_b_9to2_2, Spr_puls_val_b_1to0_2, Thru_ris_en_2, Thru_ris_tap_11to6_2, Thru_ris_tap_5to0_2, 	\
	Thru_fal_en_2, Thru_fal_tap_11to6_2, Thru_fal_tap_5to0_2, Dig_delay_2, Clk_driver_en_2, qu_reg_en_2,		\
	qq_reg_en_2, win_rst_2 , fineEn_2, fineEnb_2, rsffEn_2, dl_en_2, cp_en_2, forceCpUpb_2, forceCpDn_2,		\
	pdUpTune_1to0_2, pdDnTune_1to0_2, cpUpTune_2to0_2, cpDnTune_2to0_2, pdEn_2 , digClkPhase_7to0_2, Rst_n_async_2;	
	
	std::string L1_lup00_15to8_2; //Read-only//
	std::string L1_lup90_15to8_2;  //Read-only//
	std::string Merg_ris_fin_2; //Read-only//
	std::string Merg_fal_fin_2; //Read-only//
	std::string Decod_in_0deg; 
	std::string L1_lup00_15to8_3; 
	std::string L1_lup90_15to8_3; 
	std::string Merg_ris_fin_3; 
	std::string Merg_fal_fin_3; 
	

	int Qg00degDelay_0to4_2, Qg90degDelay_0to4_2, Qg180degDelay_0to4_2, Qg270degDelay_0to4_2, DischargeTap16_3to0;


	int ChargeTap16_3to0, DischargeTapn_3to0 , ChargeTapn16_3to0, X1sel_32to39_2;
	int X1sel_40to47_2, X2sel_32to36_2, X2sel_37to41_2, X4sel_32to36_2, X4sel_37to41_2, X8sel_32to36_2, X8sel_41_2;
	int X8sel_37to40_2, qutx_fb_180Cal_en, qutx_fb_0Cal_en, qutx_fb_180Rsff_en, qutx_fb_0Rsff_en, N, R_11to8;
	int R_7to0, Asyncrst_n, Cp_sel_6to0, Cp_sel_8to7, ForceFout, ForceFoutb, Out_en, Dll_en, Ana_en, Ngt3_3;
	int NorNdiv4_3, RorFrNpRdiv4_25to18_3, RorFrNpRdiv4_17to10_3, RorFrNpRdiv4_9to2_3, RorFrNpRdiv4_1to0_3;
	int Qu_tx_Ngt3_3, NorNdiv4_phsh_3, RorFrNpRdiv4_phsh_25to18_3, RorFrNpRdiv4_phsh_17to10_3, RorFrNpRdiv4_phsh_9to2_3;
	int RorFrNpRdiv4_phsh_1to0_3, Passthru_ref_clk_3, Byp_ram_3, Dis_adr_dith_3, Dis_p5G_dith_3, Byp_fine_3;
	int Exclude32_3, Dis_risedge_3, Dis_faledge_3, Spr_puls_en_3, Spr_puls_val_a_9to3_3, Spr_pulse_val_2to0_3;
	int Spr_puls_val_b_9to2_3, Spr_puls_val_b_1to0_3, Thru_ris_en_3, Thru_ris_tap_11to6_3, Thru_ris_tap_5to0_3;
	int Thru_fal_en_3, Thru_fal_tap_11to6_3, Thru_fal_tap_5to0_3, Dig_delay_3, Clk_driver_en_3, qu_reg_en_3;
	int qq_reg_en_3, win_rst_3, fineEn_3, fineEnb_3, rsffEn_3, dl_en_3, cp_en_3, forceCpUpb_3, forceCpDn_3;
	int pdUpTune_1to0_3, pdDnTune_1to0_3, cpUpTune_2to0_3, cpDnTune_2to0_3, pdEn_3, digClkPhase_7to0_3, Rst_n_async_3;
	int Qg00degDelay_0to4_3, Qg90degDelay_0to4_3, Qg180degDelay_0to4_3, Qg270degDelay_0to4_3, DischargeTap16_0to3_3;	
	int ChargeTap16_0to3_3, DischargeTapn_0to3_3, ChargeTapn16_0to3_3, X1sel_32to39_3, X1sel_40to47_3, X2sel_32to36_3;
	int X2sel_37to41_3, X4sel_32to36_3, X4sel_37to41_3, X8sel_32to36_3, X8sel_41_3, X8sel_37to40_3, qurx_180Cal_en;
	int qurx_0Cal_en, extClkEn, extClkEnBNOTD7, div2_rst, TxChClkSel, TxChClkEn, tx_bb_en, tx_bb_fdbk_bw;
	int tx_bb_fdbk_cal_en, tx_bb_fdbk_cart_err_en, tx_bb_fdbk_cart_fb_en, tx_bb_fdbk_cart_fwd_en, tx_bb_fdbk_en;
	int tx_bb_fdbk_1q_sel, tx_bb_fdbk_lp, tx_bb_fdbk_statt, tx_bb_fdbk_swapi, tx_bb_fdbk_swapq, tx_bb_gain_cmp;
	int tx_bb_lp, tx_bb_swapi, tx_bb_swapq, tx_butt_bw, tx_bw_trck, tx_cart_en, tx_cart_fb_bb_statt;
	int tx_cart_fb_dcoc_dac_I1, tx_cart_fb_dcoc_dac_I2, tx_cart_fb_dcoc_dac_Q1, tx_cart_fb_dcoc_dac_Q2;
	int CartesianFeedbackpathDCOCenable, CartesianFeedbackpathenable, CartesianFBpathHiResolutionDCOCenable;
	int CartesianFBpathBW, CartesianFBRFGain, CartesianFBpathSwapIandIx, CartesianFBpathSwapQandQx;
	int CartesianFBpathSwitchtoforwardSummer, tx_cart_fb_lo_select, CartesianFBpathAmp1Gain, CartesianFBpathAmp2Gain;
	int CartesianFBpathAmp3Gain, CartesianFBpathAmp4Gain, CartesianFBpathAmpCurrentSelect, CartesianFBpathZeroEnable;
	int tx_cart_zero_statt, tx_inbuf_bw, tx_inbuf_statt, tx_output_channel_sel, tx_p1_bw, tx_pw_bw1, tx_p2_bw2;		 		   		int PushPullBufferCurrent, tx_rf_aoc_bw, RFForwardPathEnable_toMUX, RFForwardPathEnable_ExternalPinenable;
	int tx_rf_fwd_lp, tx_rf_fwd_statt1, tx_rf_fwd_statt2, BBQDivideby2or4Select, BBQQuadGenEnable;
	int BBQPolyphaseQuadGenEnable, lofb_tun_s, lofb_tun_sx, lofw_tun_s2, lofw_tun_sx2, reserve_tx26, reserve_tx27;
	int rx_Idac, rx_dcs, rx_den, rx_Qdac, rx_cmpen, rx_dcoc, rx_ten, rx_ren, rx_dven, rx_dv, rx_extc, rx_cen;
	int rx_chck, rx_chcken, rx_fen, rx_onchen, rx_offchen, rx_foe, rx_offch, rx_onchf, rx_onchc, rx_qs, rx_bqg;
	int rx_rq, rx_rv, rx_rip, rx_rfp, rx_cp_12to8, rx_gs, rx_cp_7to0, rx_cv_10to3, rx_cv_2to0, rx_cc_2to0;
	int rx_cq_9to8, rx_cq_7to0, rx_lna, rx_lnab, rx_rxchen, rx_bbq_div2or4, rx_Loselect, poly_en, lorx_tun_s, lorx_tun_sx;

	std::string rx_Icmpo; //I path DCOC comparator output.  Output of the DCOC comparator - used for test purposes.  Output only.//
	std::string rx_Iodac; //I path DCOC DAC output.  Output of the DCOC DACs - used to read result of DCOC correction circuitry.  Output only.//
	std::string rx_Qcmpo; //Q path DCOC comparator output.  Output of the DCOC comparator - used for test purposes.  Output only.//
	std::string rx_Qodac; //Q path DCOC DAC output.  Output of the DCOC DACs - used to read result of DCOC correction circuitry.  Output only.//
	std::string rx_rc;

	int shft_cml_in, vagenable1, vagenable2, TestMuxBufferEnable, TestMuxEnable, TestMuxSetting, txgain;
				 
	float Foutrx,Fclk,Fouttx, Foutrxb, Foutfb;
	
	int Rst_n_async2,Rst_n_async3; //generated inside function in python code..have to be declared in c++
	// Initialize GPIO and ATR
	// GPIO are the general-purpose IO pins on the daughterboard
	// IO_RX_06 must be used for ATR (1 = TX, 0 = RX)
	// ATR is the automatic transmit/receive switching, done in the FPGA
	// FIXME 
	
	


  	usrp_basic *usrp(){
    		return d_raw_usrp;
  	}

 	rfic(usrp_basic_sptr usrp, int which);
 	~rfic();

	bool rx_write_io(int, int); //the defination may need to be changed..mask should be a hex value...will rework this
	bool rx_write_oe(int,int);
	bool rx_set_atr_rxval(int);
	bool rx_set_atr_txval(int);
	bool rx_set_atr_mask(int);


	void shutdown();
	void send_reg(int,int);

	//send register wrapper function difinations//
	void set_reg_0();
	void set_reg_1();
	void set_reg_2();
	void set_reg_3();
	void set_reg_4();
	void set_reg_5();
	void set_reg_6();
	void set_reg_7();
	void set_reg_8();
	void set_reg_9();
	void set_reg_10();
	void set_reg_12();
	void set_reg_13();
	void set_reg_14();
	void set_reg_15();
	void set_reg_16();
	void set_reg_17();
	void set_reg_18();
	void set_reg_19();
	void set_reg_20();
	void set_reg_21();
	void set_reg_22();
	void set_reg_23();
	void set_reg_24();
	void set_reg_29();
	void set_reg_30();
	void set_reg_31();
	void set_reg_32();
	void set_reg_33();
	void set_reg_34();
	void set_reg_35();
	void set_reg_36();
	void set_reg_37();
	void set_reg_38();
	void set_reg_39();
	void set_reg_40();
	void set_reg_41();
	void set_reg_42();
	void set_reg_43();
	void set_reg_48();
	void set_reg_49();
	void set_reg_50();
	void set_reg_51();
	void set_reg_52();
	void set_reg_53();
	void set_reg_54();
	void set_reg_55();
	void set_reg_56();
	void set_reg_57();
	void set_reg_58();
	void set_reg_60();
	void set_reg_61();
	void set_reg_62();
	void set_reg_63();
	void set_reg_64();
	void set_reg_65();
	void set_reg_66();
	void set_reg_67();
	void set_reg_68();
	void set_reg_69();
	void set_reg_70();
	void set_reg_71();
	void set_reg_72();
	void set_reg_77();
	void set_reg_78();
	void set_reg_79();
	void set_reg_80();
	void set_reg_81();
	void set_reg_82();
	void set_reg_83();
	void set_reg_84();
	void set_reg_85();
	void set_reg_86();
	void set_reg_87();
	void set_reg_88();
	void set_reg_89();
	void set_reg_90();
	void set_reg_91();
	void set_reg_96();
	void set_reg_97();
	void set_reg_98();
	void set_reg_99();
	void set_reg_104();
	void set_reg_105();
	void set_reg_106();
	void set_reg_107();
	void set_reg_108();
	void set_reg_109();
	void set_reg_110();
	void set_reg_111();
	void set_reg_112();
	void set_reg_113();
	void set_reg_114();
	void set_reg_116();
	void set_reg_117();
	void set_reg_118();
	void set_reg_119();
	void set_reg_120();
	void set_reg_121();
	void set_reg_122();
	void set_reg_123();
	void set_reg_124();
	void set_reg_125();
	void set_reg_126();
	void set_reg_127();
	void set_reg_128();
	void set_reg_133();
	void set_reg_134();
	void set_reg_135();
	void set_reg_136();
	void set_reg_137();
	void set_reg_138();
	void set_reg_139();
	void set_reg_140();
	void set_reg_141();
	void set_reg_142();
	void set_reg_143();
	void set_reg_144();
	void set_reg_145();
	void set_reg_146();
	void set_reg_147();
	void set_reg_152();
	void set_reg_153();
	void set_reg_156();
	void set_reg_157();
	void set_reg_158();
	void set_reg_159();
	void set_reg_160();
	void set_reg_161();
	void set_reg_162();
	void set_reg_163();
	void set_reg_164();
	void set_reg_165();
	void set_reg_166();
	void set_reg_167();
	void set_reg_168();
	void set_reg_169();
	void set_reg_170();
	void set_reg_171();
	void set_reg_172();
	void set_reg_173();
	void set_reg_174();
	void set_reg_175();
	void set_reg_176();
	void set_reg_177();
	void set_reg_178();
	void set_reg_179();
	void set_reg_180();
	void set_reg_181();
	void set_reg_192();
	void set_reg_193();
	void set_reg_194();
	void set_reg_195();
	void set_reg_196();
	void set_reg_197();
	void set_reg_198();
	void set_reg_199();
	void set_reg_200();
	void set_reg_201();
	void set_reg_202();
	void set_reg_203();
	void set_reg_204();
	void set_reg_205();
	void set_reg_206();
	void set_reg_207();
	void set_reg_220();
	void set_reg_222();
	//

	void read_reg_25();
	void read_reg_26();
	void read_reg_27();
	void read_reg_28();
	void read_reg_73();
	void read_reg_74();
	void read_reg_75();
	void read_reg_76();
	void read_reg_100();
	void read_reg_125();
	void read_reg_129();
	void read_reg_130();
	void read_reg_131();
	void read_reg_132();
	void read_reg_208();
	void read_reg_209();
	void read_reg_210();

	char get_reg(int);
	bool set_rx_gain(float);
	bool set_tx_gain(float);
	void set_fb_gain(float);
	int* calc_freq_vars(double,double);
	int* calc_phase_vars(double,double,double);
	struct freq_result_t set_rx_freq(double);
	struct freq_result_t set_tx_freq(double);
	bool set_fb_freq(double);

	bool set_rx_phase(int);
	bool set_tx_phase(int);
	
	bool set_fb_phase(int);
	bool set_rx_bw(float);
	bool set_tx_bw(float);
	void set_fb_bw(float);
	void enable_tx_fb();
	void disable_tx_fb();
	int RSSI_fade();
  	int RSSI_clip();
	 
};


/*****************************method definitions*****************************/

rfic::rfic(usrp_basic_sptr _usrp, int which)
  : d_raw_usrp(_usrp.get()), d_which(which)
{
	 Ngt3 = 0; //Output frequency control bit.  Calculated.//
	 NorNdiv4 = 1; //Output frequency control word.  Calculated.//
	 RorFrNpRdiv4_25to18 = 0; //Output frequency control word.  Calculated//
	 RorFrNpRdiv4_17to10 = 0; ////
	 RorFrNpRdiv4_9to2 = 0; ////
	 RorFrNpRdiv4_1to0 = 0; ////
	 Qu_tx_Ngt3 = 0; //Enables divide-by-4 freq divider - Phase shift control bit.  Calculated//
	 NorNdiv4_phsh = 1; //Phase shift control word.  Calculated.//
	 RorFrNpRdiv4_phsh_25to18 = 0; //Phase shift control word.  Calculated.//
	 RorFrNpRdiv4_phsh_17to10 = 0; ////
	 RorFrNpRdiv4_phsh_9to2 = 0; ////
	 RorFrNpRdiv4_phsh_1to0 = 0; ////
	 Passthru_ref_clk = 0; //A test mode where the 1 GHz input reference is passed directly to the output//
	 Byp_ram = 1; //Bypass the SRAMs//
	 Dis_adr_dith = 1; //Disable the dither generator in the ca2adr block//
	 Dis_p5G_dith = 1; //Disable the dither generator in the lup2decod block//
	 Byp_fine = 1; //Bypass fine delay line control bit//
	 Exclude32 = 0; //Bypass fine delay line control bit (exclude 32)//
	 Dis_risedge = 0; //Disable the rising edges decoders//
	 Dis_faledge = 0; //Disable the falling edges decoders//
	 Spr_puls_en = 0; //enable spur pulsing//
	 Spr_puls_val_a_9to3 = 0; //spur pulsing control word//
	 Spr_pulse_val_2to0 = 0; ////
	 Spr_puls_val_b_9to2 = 8; //spur pulsing control word//
	 Spr_puls_val_b_1to0 = 0 ;////
	 Thru_ris_en = 0; //Put rising edges decoders into through-tap mode//
	 Thru_ris_tap_11to6 = 32; //Through-tap control word//
	 Thru_ris_tap_5to0 = 0; ////
	 Thru_fal_en = 0; //Put falling edges decoders into through-tap mode//
	 Thru_fal_tap_11to6 = 32; //Through-tap control word//
	 Thru_fal_tap_5to0 = 0; ////
	 Dig_delay = 0; //This bit provides delay to the clock going into the digital block. It is a remnant of past designs and should always be left off because the digClkPhase setting in address 23 provides much finer control.//
	 Clk_driver_en = 0; //This allows the clock to reach the digital block. It first passes through the digital/analog clock synchronization mux, which means that dlEn must be on (dlEn=1) and Clk_driver=1 for the digital block to receive a clock.  See Byp_fine, address 10, bit 6//
	 qu_reg_en = 0; //This bit enables the voltage regulators that supply 1.2 V to all the analog block functions. There are 6 separate regulators that are simultaneously enabled by this bit.//
	 qq_reg_en = 0; //This bit enables the voltage regulators that supply 1.2 V to all the Quad Gen functions. There are 3 separate regulators that are simultaneously enabled by this bit.//
	 win_rst = 0; //When this bit is high, the windowing function is in a reset state, which means that no taps will be passed to the DDS output regardless of the tap select signals coming from the digital block.//
	 fineEn = 0; //This bit, when high, routes the coarse taps through the fine line before reaching the output RS Flip Flop of the DDS. When low, the coarse tap is routed directly to the output RS Flip Flop.//
	 fineEnb = 0; //Opposite of  fineEn//
	 rsffEn = 0; //This bit must be high to send the QuIET 0 and 180 degree calibration signals off chip. It does not control the RS Flip Flop outputs of the DDS, though it may have some second order (coupling) effect.//
	 dl_en = 1; //Allows the PLL reference to enter the QuIET delay line when enabled.//
	 cp_en = 1; //This bit, when enables, activates the charge pump that controls the delay line via the single pole (one capacitor) DLL loop filter.//
	 forceCpUpb = 0; //This bit only matters when pdEn=0 (address 22, bit 1). When low, the pmos device connected to the DLL loop filter cap turns on and sources current into the cap, thereby increasing the delay line control voltage. //
	 forceCpDn = 0; //This bit only matters when pdEn=0 (address 22, bit 1). When low, the nmos device connected to the DLL loop filter cap turns off and allows the pmos device to charge up the loop cap as described above.//
	 pdUpTune_1to0 = 3; //These bits control the pulse width from the phase detector into the charge up port of the charge pump. 00 turns the charge up signal off. 01 is the minimum pulse width setting and 11 is the maximum pulse width setting.//
	 pdDnTune_1to0 = 0; //These bits control the pulse width from the phase detector into the charge down port of the charge pump. 00 turns the charge down signal off. 01 is the minimum pulse width setting and 11 is the maximum pulse width setting.//
	 cpUpTune_2to0 = 7; //These bits control amount of current that is sourced while the charge up signal from the phase detector is high. 000 is minimum current and 111 is maximum current.//
	 cpDnTune_2to0 = 2; //These bits control amount of current that is sinked while the charge down signal from the phase detector is high. 000 is minimum current and 111 is maximum current.//
	 pdEn = 1; //When enables, the phase detector will send charge up and down signals to the charge pump and over ride the forceCpUp and forceCpDn settings in address 21. When disabled, the forceCpUp and forceCpDn settings will control the charge pump.//
	 digClkPhase_7to0 = 4; //Only one bit in this field should be active at one time. This signal drives a mux that selects one of eight clock phases from the delay line to drive the digital block. This is needed to control the windowing function of the DDS.//
	 Rst_n_async = 0; //Digital reset//
	
	 Qg00degDelay_0to4 = 31; //Adjusts series delay in the 0 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.//
	 Qg90degDelay_0to4 = 7; //Adjusts series delay in the 90 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.//
	 Qg180degDelay_0to4 = 31; //Adjusts series delay in the 180 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.//
	 Qg270degDelay_0to4 = 7; //Adjusts series delay in the 270 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.//
	 DischargeTap16_0to3 = 15; //Adjusts DLL offset error in the Quad Gen delay line by controlling down currents in one of the parallel charge pumps.//
	 ChargeTap16_0to3 = 4; //Adjusts DLL offset error in the Quad Gen delay line by controlling up currents in one of the parallel charge pumps.//
	 DischargeTapn_0to3 = 15; //Adjusts DLL offset error in the Quad Gen delay line by controlling down currents in one of the parallel charge pumps.//
	 ChargeTapn16_0to3 = 2; //Adjusts DLL offset error in the Quad Gen delay line by controlling up currents in one of the parallel charge pumps.//
	 X1sel_32to39 = 0; //Control for the divide-by-two and x1 functions.//
	 X1sel_40to47 = 0; //Control for the divide-by-two and x1 functions.//
	 X2sel_32to36 = 0; //Control for the x2 function.//
	 X2sel_37to41 = 0; //Control for the x2 function.//
	 X4sel_32to36 = 0; //Control for the x4 function.//
	 X4sel_37to41 = 0; //Control for the x4 function.//
	 X8sel_32to36 = 0; //Bit 41 is used for a fine line windowing control bit. If the fine line is needed, this bit needs to be set high if Fout is close to Fref (greater than ~ 950 MHz) or for some testing modes like pass through or thru_rise_en.//
	 X8sel_41 = 0; //hiFout - set for passthrough and Fout close to Fref//
	 X8sel_37to40 = 0; ////
	 qutx_fwd_180Cal_en = 0; //Enables the pad driver that sends the falling edge signal off chip. This falling edge signal is used internally to trigger the 'Reset' pin of the output RS Flip Flop.//
	 qutx_fwd_0Cal_en = 0; //Enables the pad driver that sends the rising edge signal off chip. This rising edge signal is used internally to trigger the 'Set' pin of the output RS Flip Flop.//
		//-------------------------------------------------------------------------------------------------------
		// TRANSMIT FEEDBACK QuIET FREQUENCY GENERATOR
		//-------------------------------------------------------------------------------------------------------
	 Ngt3_2 = 0; //Output frequency control bit.  Calculated.//
	 NorNdiv4_2 = 1; //Output frequency control word.  Calculated.//
	 RorFrNpRdiv4_25to18_2 = 0; //Output frequency control word.  Calculated.//
	 RorFrNpRdiv4_17to10_2 = 0; ////
	 RorFrNpRdiv4_9to2_2 = 0; ////
	 RorFrNpRdiv4_1to0_2 = 0; ////
	 Qu_tx_Ngt3_2 = 0; //Enables divide-by-4 freq divider - Phase shift control bit.  Calculated//
	 NorNdiv4_phsh_2 = 1; //Phase shift control word.  Calculated//
	 RorFrNpRdiv4_phsh_25to18_2 = 0; //Phase shift control word.  Calculated//
	 RorFrNpRdiv4_phsh_17to10_2 = 0; ////
	 RorFrNpRdiv4_phsh_9to2_2 = 0; ////
	 RorFrNpRdiv4_phsh_1to0_2 = 0 ;////
	 Passthru_ref_clk_2 = 0; //Enable reference clock pass-through mode//
	 Byp_ram_2 = 1; //Bypass the SRAMs//
	 Dis_adr_dith_2 = 1; //Disable the dither generator in the ca2adr block//
	 Dis_p5G_dith_2 = 1; //Disable the dither generator in the lup2decod block//
	 Byp_fine_2 = 1; //Bypass fine delay line control bit//
	 Exclude32_2 = 0; //Bypass fine delay line control bit (exclude 32)//
	 Dis_risedge_2 = 0; //Disable the rising edges decoders//
	 Dis_faledge_2 = 0;//Disable the falling edges decoders//
	 Spr_puls_en_2 = 0; //Enable spur pulsing mode//
	 Spr_puls_val_a_9to3_2 = 0; //Spur pulsing mode control word//
	 Spr_pulse_val_2to0_2 = 0; ////
	 Spr_puls_val_b_9to2_2 = 8; //Spur pulsing mode control word//
	 Spr_puls_val_b_1to0_2 = 0 ;////
	 Thru_ris_en_2 = 0; //Put rising edges decoders into through-tap mode//
	 Thru_ris_tap_11to6_2 = 32; //Through-tap mode control word//
	 Thru_ris_tap_5to0_2 = 0; //Through-tap mode control word//
	 Thru_fal_en_2 = 0; //Put falling edges decoders into through-tap mode//
	 Thru_fal_tap_11to6_2 = 32; //Through-tap mode control word//
	 Thru_fal_tap_5to0_2 = 0; //Through-tap mode control word//
	 Dig_delay_2 = 0; //This bit provides delay to the clock going into the digital block. It is a remnant of past designs and should always be left off because the digClkPhase setting in address 23 provides much finer control.//
	 Clk_driver_en_2 = 0; //This bit provides delay to the clock going into the digital block. It is a remnant of past designs and should always be left off because the digClkPhase setting in address 23 provides much finer control.  See Byp_fine, address 10, bit 6//
	 qu_reg_en_2 = 0; //This bit enables the voltage regulators that supply 1.2 V to all the analog block functions. There are 6 separate regulators that are simultaneously enabled by this bit.//
	 qq_reg_en_2 = 0; //This bit enables the voltage regulators that supply 1.2 V to all the Quad Gen functions. There are 3 separate regulators that are simultaneously enabled by this bit.//
	 win_rst_2 = 0; //When this bit is high, the windowing function is in a reset state, which means that no taps will be passed to the DDS output regardless of the tap select signals coming from the digital block.//
	 fineEn_2 = 0; //This bit, when high, routes the coarse taps through the fine line before reaching the output RS Flip Flop of the DDS. When low, the coarse tap is routed directly to the output RS Flip Flop.//
	 fineEnb_2 = 0; //Opposite of  fineEn.//
	 rsffEn_2 = 0; //This bit must be high to send the QuIET 0 and 180 degree calibration signals off chip. It does not control the RS Flip Flop outputs of the DDS, though it may have some second order (coupling) effect.//
	 dl_en_2 = 1; //Allows the PLL reference to enter the QuIET delay line when enabled.//
	 cp_en_2 = 1; //This bit, when enables, activates the charge pump that controls the delay line via the single pole (one capacitor) DLL loop filter.//
	 forceCpUpb_2 = 0; //This bit only matters when pdEn=0 (address 22, bit 1). When low, the pmos device connected to the DLL loop filter cap turns on and sources current into the cap, thereby increasing the delay line control voltage. //
	 forceCpDn_2 = 0; //This bit only matters when pdEn=0 (address 22, bit 1). When low, the nmos device connected to the DLL loop filter cap turns off and allows the pmos device to charge up the loop cap as described above.//
	 pdUpTune_1to0_2 = 3; //These bits control the pulse width from the phase detector into the charge up port of the charge pump. 00 turns the charge up signal off. 01 is the minimum pulse width setting and 11 is the maximum pulse width setting.//
	 pdDnTune_1to0_2 = 0 ;//These bits control the pulse width from the phase detector into the charge down port of the charge pump. 00 turns the charge down signal off. 01 is the minimum pulse width setting and 11 is the maximum pulse width setting.//
	 cpUpTune_2to0_2 = 7; //These bits control amount of current that is sourced while the charge up signal from the phase detector is high. 000 is minimum current and 111 is maximum current.//
	 cpDnTune_2to0_2 = 2; //These bits control amount of current that is sinked while the charge down signal from the phase detector is high. 000 is minimum current and 111 is maximum current.//
	 pdEn_2 = 1; //When enables, the phase detector will send charge up and down signals to the charge pump and over ride the forceCpUp and forceCpDn settings in address 21. When disabled, the forceCpUp and forceCpDn settings will control the charge pump.//
	 digClkPhase_7to0_2 = 4; //Only one bit in this field should be active at one time. This signal drives a mux that selects one of eight clock phases from the delay line to drive the digital block. This is needed to control the windowing function of the DDS.//
	 Rst_n_async_2 = 0; //Digital reset//

	 Qg00degDelay_0to4_2 = 31; // Adjusts series delay in the 0 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.// 
	 Qg90degDelay_0to4_2 = 7; // // Adjusts series delay in the 90 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.
	 Qg180degDelay_0to4_2 = 31; // Adjusts series delay in the 180 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.// 
	 Qg270degDelay_0to4_2 = 7; // Adjusts series delay in the 270 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.// 
	 DischargeTap16_3to0 = 15; // Adjusts DLL offset error in the Quad Gen delay line by controlling down currents in one of the parallel charge pumps.// 
	 ChargeTap16_3to0 = 4; // Adjusts DLL offset error in the Quad Gen delay line by controlling up currents in one of the parallel charge pumps.// 
	 DischargeTapn_3to0 = 15; // Adjusts DLL offset error in the Quad Gen delay line by controlling down currents in one of the parallel charge pumps.// 
	 ChargeTapn16_3to0 = 2; // Adjusts DLL offset error in the Quad Gen delay line by controlling up currents in one of the parallel charge pumps.// 
	 X1sel_32to39_2 = 0; // Control for the divide-by-two and x1 functions.// 
	 X1sel_40to47_2 = 0 ;// Control for the divide-by-two and x1 functions.// 
	 X2sel_32to36_2 = 0; // Control for the x2 function.// 
	 X2sel_37to41_2 = 0; // Control for the x2 function.// 
	 X4sel_32to36_2 = 0; // Control for the x4 function.// 
	 X4sel_37to41_2 = 0; // Control for the x4 function.// 
	 X8sel_32to36_2 = 0; // Bit 41 is used for a fine line windowing control bit. If the fine line is needed, this bit needs to be set high if Fout is close to Fref (greater than ~ 950 MHz) or for some testing modes like pass through or thru_rise_en.// 
	 X8sel_41_2 = 0; // hiFout - set for passthrough and Fout close to Fref// 
	 X8sel_37to40_2 = 0; // // 
	 qutx_fb_180Cal_en = 0; // Enables the pad driver that sends the falling edge signal off chip. This falling edge signal is used internally to trigger the 'Reset' pin of the output RS Flip Flop.// 
	 qutx_fb_0Cal_en = 0; // Enables the pad driver that sends the rising edge signal off chip. This rising edge signal is used internally to trigger the 'Set' pin of the output RS Flip Flop.// 
	 qutx_fb_180Rsff_en = 0; // off// 
	 qutx_fb_0Rsff_en = 0; // off// 
	// -------------------------------------------------------------------------------------------------------
	//  QuIET Dm
	// -------------------------------------------------------------------------------------------------------
	 N = 4; // // 
	 R_11to8 = 13; // // 
	 R_7to0 = 172; // // 
	 Asyncrst_n = 0; // off// 
	 Cp_sel_6to0 = 63; // // 
	 Cp_sel_8to7 = 0; // // 
	 ForceFout = 0; // off// 
	 ForceFoutb = 0; // off// 
	 Out_en = 0; // off// 
	 Dll_en = 1; // on// 
	 Ana_en = 1; // off// 
	
	// -------------------------------------------------------------------------------------------------------
	//  RECEIVE QuIET FREQUENCY GENERATOR
	// -------------------------------------------------------------------------------------------------------
	 Ngt3_3 = 0; // Output frequency control bit.  Calculated.// 
	 NorNdiv4_3 = 0; // Output frequency control word.  Calculated.// 
	 RorFrNpRdiv4_25to18_3 = 0; // Output frequency control word.  Calculated.// 
	 RorFrNpRdiv4_17to10_3 = 0; // // 
	 RorFrNpRdiv4_9to2_3 = 0; // // 
	 RorFrNpRdiv4_1to0_3 = 0; // // 
	 Qu_tx_Ngt3_3 = 0; // Enables divide-by-4 freq divider - Phase shift control bit.  Calculated.// 
	 NorNdiv4_phsh_3 = 1; // Phase shift control word.  Calculated// 
	 RorFrNpRdiv4_phsh_25to18_3 = 0; // Phase shift control word.  Calculated.// 
	 RorFrNpRdiv4_phsh_17to10_3 = 0; // // 
	 RorFrNpRdiv4_phsh_9to2_3 = 0; // // 
	 RorFrNpRdiv4_phsh_1to0_3 = 0; // // 
	 Passthru_ref_clk_3 = 0; // Enable reference clock pass-through mode// 
	 Byp_ram_3 = 1; // Bypass the SRAMs// 
	 Dis_adr_dith_3 = 1; // Disable the dither generator in the ca2adr block// 
	 Dis_p5G_dith_3 = 1; // Disable the dither generator in the lup2decod block// 
	 Byp_fine_3 = 1; // Bypass fine delay line control bit// 
	 Exclude32_3 = 0; // Bypass fine delay line control bit (exclude 32)// 
	 Dis_risedge_3 = 0; // Disable the rising edges decoders// 
	 Dis_faledge_3 = 0; // Disable the falling edges decoders// 
	 Spr_puls_en_3 = 0; // Enable spur pulsing mode// 
	 Spr_puls_val_a_9to3_3 = 0; // Spur pulsing mode control word// 
	 Spr_pulse_val_2to0_3 = 0; // // 
	 Spr_puls_val_b_9to2_3 = 8; // Spur pulsing mode control word// 
	 Spr_puls_val_b_1to0_3 = 0; // // 
	 Thru_ris_en_3 = 0; // Put rising edges decoders into through-tap mode// 
	 Thru_ris_tap_11to6_3 = 32; // Through-tap mode control word// 
	 Thru_ris_tap_5to0_3 = 0; // Through-tap mode control word// 
	 Thru_fal_en_3 = 0; // Put falling edges decoders into through-tap mode// 
	 Thru_fal_tap_11to6_3 = 0; // Through-tap mode control word// 
	 Thru_fal_tap_5to0_3 = 0; // Through-tap mode control word// 
	 Dig_delay_3 = 0; // This bit provides delay to the clock going into the digital block. It is a remnant of past designs and should always be left offbecause the digClkPhase setting in address 23 provides much finer control.// 
	 Clk_driver_en_3 = 0; // This allows the clock to reach the digital block. It first passes through the digital/analog clock synchronization mux, which means that dlEn must be on (dlEn=1) and Clk_driver=1 for the digital block to receive a clock.  See Byp_fine, address 10, bit 6// 
	 qu_reg_en_3 = 0; // This bit enables the voltage regulators that supply 1.2 V to all the analog block functions. There are 6 separate regulators that are simultaneously enabled by this bit.// 
	 qq_reg_en_3 = 0; // This bit enables the voltage regulators that supply 1.2 V to all the Quad Gen functions. There are 3 separate regulators that are simultaneously enabled by this bit.// 
	 win_rst_3 = 0; // When this bit is high, the windowing function is in a reset state, which means that no taps will be passed to the DDS output regardless of the tap select signals coming from the digital block.// 
	 fineEn_3 = 0; // This bit, when high, routes the coarse taps through the fine line before reaching the output RS Flip Flop of the DDS. When low, the coarse tap is routed directly to the output RS Flip Flop.// 
	 fineEnb_3 = 0; // Opposite of  fineEn.// 
	 rsffEn_3 = 0; // This bit must be high to send the QuIET 0 and 180 degree calibration signals off chip. It does not control the RS Flip Flop outputs of the DDS, though it may have some second order (coupling) effect.// 
	 dl_en_3 = 1; // Allows the PLL reference to enter the QuIET delay line when enabled.// 
	 cp_en_3 = 1; // This bit, when enables, activates the charge pump that controls the delay line via the single pole (one capacitor) DLL loop filter.// 
	 forceCpUpb_3 = 0; // This bit only matters when pdEn=0 (address 22, bit 1). When low, the pmos device connected to the DLL loop filter cap turns on and sources current into the cap, thereby increasing the delay line control voltage. // 
	 forceCpDn_3 = 0; // This bit only matters when pdEn=0 (address 22, bit 1). When low, the nmos device connected to the DLL loop filter cap turns off and allows the pmos device to charge up the loop cap as described above.// 
	 pdUpTune_1to0_3 = 3; // These bits control the pulse width from the phase detector into the charge up port of the charge pump. 00 turns the charge up signal off. 01 is the minimum pulse width setting and 11 is the maximum pulse width setting.// 
	 pdDnTune_1to0_3 = 1; // These bits control the pulse width from the phase detector into the charge down port of the charge pump. 00 turns the charge down signal off. 01 is the minimum pulse width setting and 11 is the maximum pulse width setting.// 
	 cpUpTune_2to0_3 = 7; // These bits control amount of current that is sourced while the charge up signal from the phase detector is high. 000 is minimum current and 111 is maximum current.// 
	 cpDnTune_2to0_3 = 2; // These bits control amount of current that is sinked while the charge down signal from the phase detector is high. 000 is minimum current and 111 is maximum current.// 
	 pdEn_3 = 1; // When enables, the phase detector will send charge up and down signals to the charge pump and over ride the forceCpUp and forceCpDn settings in address 21. When disabled, the forceCpUp and forceCpDn settings will control the charge pump.// 
	 digClkPhase_7to0_3 = 4; // Only one bit in this field should be active at one time. This signal drives a mux that selects one of eight clock phases from the delay line to drive the digital block. This is needed to control the windowing function of the DDS.// 
	 Rst_n_async_3 = 0; // Digital reset.// 
	 Qg00degDelay_0to4_3 = 31; // Adjusts series delay in the 0 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.// 
	 Qg90degDelay_0to4_3 = 31; // Adjusts series delay in the 90 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.// 
	 Qg180degDelay_0to4_3 = 31; // Adjusts series delay in the 180 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.// 
	 Qg270degDelay_0to4_3 = 31; // Adjusts series delay in the 270 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.// 
	 DischargeTap16_0to3_3 = 15; // Adjusts DLL offset error in the Quad Gen delay line by controlling down currents in one of the parallel charge pumps.// 
	 ChargeTap16_0to3_3 = 15; // Adjusts DLL offset error in the Quad Gen delay line by controlling up currents in one of the parallel charge pumps.// 
	 DischargeTapn_0to3_3 = 15; // Adjusts DLL offset error in the Quad Gen delay line by controlling down currents in one of the parallel charge pumps.// 
	 ChargeTapn16_0to3_3 = 15; // Adjusts DLL offset error in the Quad Gen delay line by controlling up currents in one of the parallel charge pumps.// 
	 X1sel_32to39_3 = 0; // Control for the divide-by-two and x1 functions.// 
	 X1sel_40to47_3 = 0; // Control for the divide-by-two and x1 functions.// 
	 X2sel_32to36_3 = 0; // Control for the x2 function.// 
	 X2sel_37to41_3 = 0; // Control for the x2 function.// 
	 X4sel_32to36_3 = 0; // Control for the x4 function.// 
	 X4sel_37to41_3 = 0; // Control for the x4 function.// 
	 X8sel_32to36_3 = 0; // Bit 41 is used for a fine line windowing control bit. If the fine line is needed, this bit needs to be set high if Fout is close to Fref (greater than ~ 950 MHz) or for some testing modes like pass through or thru_rise_en.// 
	 X8sel_41_3 = 0; // hiFout - set for passthrough and Fout close to Fref// 
	 X8sel_37to40_3 = 0; // // 
	 qurx_180Cal_en = 0; // Enables the pad driver that sends the falling edge signal off chip. This falling edge signal is used internally to trigger the 'Reset' pin of the output RS Flip Flop.// 
	 qurx_0Cal_en = 0; // Enables the pad driver that sends the rising edge signal off chip. This rising edge signal is used internally to trigger the 'Set' pin of the output RS Flip Flop.// 
	// -------------------------------------------------------------------------------------------------------
	//  PLL
	// -------------------------------------------------------------------------------------------------------
	 extClkEn = 0; // PLL Reg 0// 
	 extClkEnBNOTD7 = 1; // on// 
	 div2_rst = 1; // on// 
	 TxChClkSel = 0; // // 
	 TxChClkEn = 0; // PLL Reg 1// 
	// -------------------------------------------------------------------------------------------------------
	//  TRANSMITTER
	// -------------------------------------------------------------------------------------------------------
	 tx_bb_en = 0; // BB Fdbk Mux Buffer BW Control.  Enables the Forward BB Reference Section of TX// 
	 tx_bb_fdbk_bw = 0; // Sets the BW of the BB Correction feedback amp// 
	 tx_bb_fdbk_cal_en = 0; // BB Feedback Mux path Routing.  Shorts the BB Correction feedback Amp input for self-calibration// 
	 tx_bb_fdbk_cart_err_en = 0; // Routes the Cartesian error signal through the BB Correction feedback// 
	 tx_bb_fdbk_cart_fb_en = 0; // Routes the Cartesian feedback signal through the BB Correction feedback// 
	 tx_bb_fdbk_cart_fwd_en = 0; // Routes the Cartesian reference signal through the BB Correction feedback// 
	 tx_bb_fdbk_en = 0; // BB Feedback Mux path Routing.  Enables the BB Correction feedback path via the RX pins// 
	 tx_bb_fdbk_1q_sel = 0; // Chooses between I or Q channel for the BB Correction feedback path// 
	 tx_bb_fdbk_lp = 0; // BB Fdbk Mux Buffer current.  Sets the current drive capability for BB Correction feedback Amp// 
	 tx_bb_fdbk_statt = 3; // BB Fdbk Mux Buffer Gain Control.  BB Feedback Attenuator.  Sets the voltage gain for BB Correction feedback Amp// 
	 tx_bb_fdbk_swapi = 0; // Baseband Feedback Swap I & Ix.  Swaps the I and Ix BB signals through the BB Correction feedback path// 
	 tx_bb_fdbk_swapq = 0; // Baseband feedback Swap Q & Qx.  Swaps the Q and Qx BB signal through the BB Correction feedback path// 
	 tx_bb_gain_cmp = 1; // Baseband Gain 1 dB Compensation.  Adds and extra 1.3 dB of Forward Baseband Reference Gain// 
	 tx_bb_lp = 0; // BB ref. stage current.  BB Amp Stage Current.  Sets the current drive capability for Forward BB Reference Amps// 
	 tx_bb_swapi = 1; // Baseband Swap I & Ix.  Swaps the I and Ix BB signals through the Forward BB Reference Path// 
	 tx_bb_swapq = 0; // Baseband Swap Q & Qx.  Swaps the Q and Qx BB signals through the Forward BB Reference Path// 
	 tx_butt_bw = 0; // BB ref. Butterworth filter BW control.  Sets the BW of the Forward BB Reference 4-pole Butterworth Filters// 
	 tx_bw_trck = 5; // TX MIM cap tracking filter BW.  Bandwidth Tracking.  Sets tracking BW of all the MIM cap based TX Filters (16 states)// 
	 tx_cart_en = 0; // Cartesian FB path Enable.  Enables the Cartesian Baseband Section of Tx// 
	 tx_cart_fb_bb_statt = 15; // Cartesian down-mix path BB gain.  Cartesian FB path BB gain.  Sets the voltage gain for Cartesian BB down converter PMA// 
	 tx_cart_fb_dcoc_dac_I1 = 32; // Sets Cartesian BB down converter PMA Dc offset correction DAC I1// 
	 tx_cart_fb_dcoc_dac_I2 = 32; // Sets Cartesian BB down converter PMA Dc offset correction DAC I2// 
	 tx_cart_fb_dcoc_dac_Q1 = 32; // Sets Cartesian BB down converter PMA Dc offset correction DAC Q1// 
	 tx_cart_fb_dcoc_dac_Q2 = 32; // Sets Cartesian BB down converter PMA Dc offset correction DAC Q2// 
	 CartesianFeedbackpathDCOCenable = 0; // Cartesian down-mix path BB BW// 
	 CartesianFeedbackpathenable = 0; // off// 
	 CartesianFBpathHiResolutionDCOCenable = 0; // off// 
	 CartesianFBpathBW = 15; // // 
	 CartesianFBRFGain = 0; // Cartesian down conv. path RF Gain// 
	 CartesianFBpathSwapIandIx = 0; // Swap I & Ix BB in Down Converter// 
	 CartesianFBpathSwapQandQx = 0; // off// 
	 CartesianFBpathSwitchtoforwardSummer = 0; // off// 
	 tx_cart_fb_lo_select = 0; // Cart. down conv LO curr. (tied to Gain)// 
	 CartesianFBpathAmp1Gain = 3; // // 
	 CartesianFBpathAmp2Gain = 3; // // 
	 CartesianFBpathAmp3Gain = 3; // // 
	 CartesianFBpathAmp4Gain = 3; // // 
	 CartesianFBpathAmpCurrentSelect = 7; // // 
	 CartesianFBpathZeroEnable = 0; // off// 
	 tx_cart_zero_statt = 1; // Cartesian FB path Zero Gain.  Sets the voltage gain for Cartesian Forward BB Zero Amp// 
	 tx_inbuf_bw = 0; // Sets the BW of the Forward BB Reference Input Buffers// 
	 tx_inbuf_statt = 0; // Sets the attenuation of the Forward BB Ref. Buffers// 
	 tx_output_channel_sel = 0; // Selects from the 3 RF Forward TX output paths, 000 is full power down// 
	 tx_p1_bw = 0; // Sets the BW of the Cartesian Forward BB Loop Pole 1// 
	 tx_pw_bw1 = 0; // Cartesian FB path Pole 2 Bandwidth.  Sets the BW of the Cartesian Forward BB Loop Pole 2// 
	 tx_p2_bw2 = 0; // Cartesian FB path Pole 2 Bandwidth.  Sets the BW of the Cartesian Forward BB Loop Pole 2// 
	 PushPullBufferCurrent = 7; // // 
	 tx_rf_aoc_bw = 0; // Sets the BW of the AOC control line// 
	 RFForwardPathEnable_toMUX = 0; // off// 
	 RFForwardPathEnable_ExternalPinenable = 1; // on// 
	 tx_rf_fwd_lp = 0; // RF Forward Bias Reference Control.  RF Forward Path Current Drain Select.  Sets the current drive capability for Forward RF Output Drivers// 
	 tx_rf_fwd_statt1 = 0; // RF Passive Step Attenuator control.  RF Forward Path Step Attn1.  Sets the attenuation level for the RF Step attenuators// 
	 tx_rf_fwd_statt2 = 0; // RF Output Driver Step Attn. Control.  RF Forward Path Step Attn2.  Sets the attenuation level for the RF Output Drivers// 
	 BBQDivideby2or4Select = 0; // BBQ Quad Gen Divide by 2 or 4 (High=1/4)// 
	 BBQQuadGenEnable = 0; // Bypass Quiet LO with external LO// 
	 BBQPolyphaseQuadGenEnable = 0; // off// 
	 lofb_tun_s = 8; // // 
	 lofb_tun_sx = 8; // // 
	 lofw_tun_s2 = 8; // // 
	 lofw_tun_sx2 = 8; // // 
	 reserve_tx26 = 0; // // 
	 reserve_tx27 = 0; // // 
	// -------------------------------------------------------------------------------------------------------
	//  RECEIVER
	// -------------------------------------------------------------------------------------------------------
	 rx_Idac = 16; // I path DCOC DAC setting.  Digital values for the DC offset adjustment.  11111 represents the maximum positive offset adjust and 00000 represents the maximum negative offset adjust.  By design, codes 10000 and 01111 cause no change in the offset voltage.// 
	 rx_dcs = 0; // DCOC step size select.  Selects the proper current reference in the DAC to maintain constant step size at ouptut of baseband filters.  This value works in tandem with the BiQuad Gain Select (address 198, bits 4:3) to maintain a constant step size of 24 mV at filter output.// 
	 rx_den = 0; // Enables the DC offset correction DACs in the I and Q path.// 
	 rx_Qdac = 12; // Q path DCOC DAC setting.  Digital values for the DC offset adjustment.  11111 represents the maximum positive offset adjust and 00000 represents the maximum negative offset adjust.  By design, codes 10000 and 01111 cause no change in the offset voltage.// 
	 rx_cmpen = 0; // Enables the DC offset correction comparator used in the DCOC circuitry.// 
	 rx_dcoc = 0; // Enables the DC offset correction circuitry for automatic correction of the DC offset in the baseband filters.// 
	 rx_ten = 0; // Enables the RC tuning circuit to tune RX and TX baseband filters.// 
	 rx_ren = 0; // Enables the ramp circuit used in the RC tuning circuitry to tune the RX and TX baseband filters.// 
	 rx_dven = 0; // // 
	 rx_dv = 0; // DCOC/tune clock divider select.  Selects the clock rate used for clocking the DCOC and RC tuning circuit.  Bits 3 and 2 set the divider setting used for both the DCOC circuitry and RC Tune circuitry.  Bits 1 and 0 set the divider setting for the dedicated divider in the DCOC circuitry.  Table below shows the mapping of divider settings.  The DCOC clock divider setting is the total divide ratio of both dividers.  The maximum divide ratio is 8*8 = 64.// 
	 rx_extc = 0; // Enables the external capacitor pins to allow for external low-frequency pole to be placed in the signal path between the mixer and baseband filter.// 
	 rx_cen = 0; // Chopper enable for filter stages.  Settings to enable which amplifier the clock is being applied// 
	 rx_chck = 0; // Divider setting for the chopper clock// 
	 rx_chcken = 0; // Enables the baseband filter chopper clock.// 
	 rx_fen = 0; // Enables baseband filters.  0 puts filter in power save mode.// 
	 rx_onchen = 0; // Enables on-channel detector.// 
	 rx_offchen = 0; // Enables off-channel detector// 
	 rx_foe = 0; // Enables the output of the baseband filters.  Otherwise the baseband filter outputs are in a Hi-Z state to allow transmitter to use filter output pins.  When Filter Enable is set LOW, outputs are disabled (Hi-Z)// 
	 rx_offch = 1; // Sets the Clip Threshold for the Off-channel Detector// 
	 rx_onchf = 0; // Sets the Fade Threshold for the On-channel Detector relative to the On-channel clip point.// 
	 rx_onchc = 2 ;// Sets the Clip Threshold for the On-channel Detector// 
	 rx_qs = 0; // Sets the BiQuad filter Q// 
	 rx_bqg = 0; // Set BiQuad filter gain// 
		// FIXME Maybe set rx_rq to 0
	 rx_rq = 1; // Sets the BiQuad filter resistor value.  The natural frequency of the BiQuad (wo) is this resistor value multiplied by the BiQuad Capacitor value.// 
	 rx_rv = 1; // Sets the VGA filter (passive filter) resistor value.  The pole frequency of the passive filter is this resistor value multiplied by the VGA capacitor value.// 
	 rx_rip = 0; // Sets the MPA input resistor value that sets the gain of the PMA.  Gain of the PMA is Rf/Rin where Rf is the PMA feedback resistor and Rin is the input resistor.  Note that the input resistance remains at 2 kohm differential for all settings.  An R2R ladder is used to accomplish this while changing the Rin value.// 
	 rx_rfp = 2; // Sets the PMA feedback resistor value that sets the gain of the PMA as well as the pole frequency (along with PMA capacitor value).  Gain of the PMA is Rf/Rin where Rf is the PMA feedback resistor and Rin is the input resistor.// 
	 rx_cp_12to8 = 0; // Sets the PMA filter capacitor value.  The pole frequency of the PMA filter is the PMA feedback resistor value multiplied by this Capacitor value.  PMA Capacitor (in pF) = (PMAC) * 0.0625 + 1// 
	 rx_gs = 0; // Sets the gain of the VGA in the baseband filter// 
	 rx_cp_7to0 = 0; // PMA cap select LSBs.  Sets the PMA filter capacitor value.  The pole frequency of the PMA filter is the PMA feedback resistor value multiplied by this Capacitor value.  PMA Capacitor (in pF) = (PMAC) * 0.0625 + 1// 
	 rx_cv_10to3 = 0; // VGA cap select MSBs.  Sets the VGA (passive) filter capacitor value.  This pole frequency of the passive filter is the VGA resistor value multiplied by this Capacitor value.  VGA Capacitor (in pF) = (VGAC) * 0.0625 + 1// 
	 rx_cv_2to0 = 0; // VGA cap select LSBs.  Sets the VGA (passive) filter capacitor value.  This pole frequency of the passive filter is the VGA resistor value multiplied by this Capacitor value.  VGA Capacitor (in pF) = (VGAC) * 0.0625 + 1// 
	 rx_cc_2to0 = 0; // Compensation control.  Disables additional compensation capacitance in the VGA and BiQuad op-amps to allow for higher bandwidths.  Also increases the op-ampdominate pole-frequency which improves filter response.  Bit 4 controls the VGA amplifier, Bit 3 controls the feedback amplifier in the BiQuad, and Bit 2 controls the output buffer in the BiQuad.// 
	 rx_cq_9to8 = 0; // BiQuad cap select MSBs.  Sets the BiQuad filter capacitor value.  The natural frequency of the BiQuad (wo) is the BiQuad resistor value multiplied by this Capacitor value.  BiQuad Capacitor (in pF) = (BiQuadC) * 0.125 + 2// 
	 rx_cq_7to0 = 0; // BiQuad cap select LSBs.  Sets the BiQuad filter capacitor value.  The natural frequency of the BiQuad (wo) is the BiQuad resistor value multiplied by this Capacitor value.  BiQuad Capacitor (in pF) = (BiQuadC) * 0.125 + 2// 
	 rx_lna = 1; // LNA select// 
	 rx_lnab = 0; // LNA bias select// 
	 rx_rxchen = 0; // RX mixer enable.  Must be set to 1 to enable Mixer operation// 
	 rx_bbq_div2or4 = 0; // Selects divide ratio of RX Quad Gen when using external LO.  0->DIV2, 1 ->DIV1// 
	 rx_Loselect = 0; // RX external LO select.  Enables external LO clock source// 
	 poly_en = 0; // off// 
	 lorx_tun_s = 8; // // 
	 lorx_tun_sx = 8; // // 

	 shft_cml_in = 0; //Enable - 150mV level shift of Ref. BB VAG//
	 vagenable1 = 1; //Enable VAG Gen into Sleep Mode (slow ramp up)//
	 vagenable2 = 1; //Enable VAG Gen in Full On Mode (Fast ramp from sleep)//
	//-------------------------------------------------------------------------------------------------------
	// TEST MULTIPLEXER
	//-------------------------------------------------------------------------------------------------------
	 TestMuxBufferEnable = 0; //Enable Test Mux Buffer//
	 TestMuxEnable = 0; //Enable Test Mux//
	 TestMuxSetting = 0; //Four Output Description (Test1, Test2, Test3, Test4)//
	 txgain = 0;	//Set Transmit Gain//
	 Fclk = 1000e6; //Default clock frequency, in Hz// 

	 Fouttx = 0;	// Default tx frequency is zero//
	 Foutrx = 0;	// Default rx frequency is zero//
	 Foutfb = 0; // Default feedback frequency is zero//

	// Initialize GPIO and ATR
	// GPIO are the general-purpose IO pins on the daughterboard
	// IO_RX_06 must be used for ATR (1 = TX, 0 = RX)
	// ATR is the automatic transmit/receive switching, done in the FPGA
	// FIXME 


	rx_write_io(0, TX_EN);
	rx_write_oe(TX_EN, TX_EN);
	rx_set_atr_rxval(0);
	rx_set_atr_txval(TX_EN);
	rx_set_atr_mask(TX_EN);	

	d_spi_format = SPI_FMT_MSB | SPI_FMT_HDR_2;
	d_spi_format_no_header = SPI_FMT_MSB | SPI_FMT_HDR_0;

	if (int(which) == 0)
		d_spi_enable = SPI_ENABLE_RX_A;
	else
		d_spi_enable = SPI_ENABLE_RX_B;

}

rfic::~rfic()
{
  //printf("rfic::destructor\n");
  shutdown();
}

void
rfic::shutdown()
{
	Rst_n_async = 0;
	set_reg_24();
	Rst_n_async2 = 0;
	set_reg_72();
	Rst_n_async3 = 0;
	set_reg_128();

	X1sel_32to39_3 = 0;
	X1sel_40to47_3 = 0;
	X2sel_32to36_3 = 0;
	X2sel_37to41_3 = 0;
	X4sel_32to36_3 = 0;
	X4sel_37to41_3 = 0;

	set_reg_139();
	set_reg_140();
	set_reg_141();
	set_reg_142();
	set_reg_143();
	set_reg_144();

	X1sel_32to39 = 0;
	X1sel_40to47 = 0;
	X2sel_32to36 = 0;
	X2sel_37to41 = 0;
	X4sel_32to36 = 0;
	X4sel_37to41 = 0;

	set_reg_35();
	set_reg_36();
	set_reg_37();
	set_reg_38();
	set_reg_39();
	set_reg_40();

	X1sel_32to39_2 = 0;
	X1sel_40to47_2 = 0;
	X2sel_32to36_2 = 0;
	X2sel_37to41_2 = 0;
	X4sel_32to36_2 = 0;
	X4sel_37to41_2 = 0;

	set_reg_83();
	set_reg_84();
	set_reg_85();
	set_reg_86();
	set_reg_87();
	set_reg_88();

	// --------------------------------------------------------------------
	// These methods set the RFIC onboard registers over the SPI bus.
	// Thus, the shift values here are the 0-7 values from the data sheet

	// For more information about setting each variable and SPI register, see RFIC4 SPI Default Variables.xls 


///////////////////////////////////have to do this  after i finish all the set reg functions///////////////////////////////////
}

bool
rfic::rx_write_oe(int value,int mask){

	if (int(d_which) == 0)
		return usrp()->_write_fpga_reg(FR_OE_1, (mask << 16) | value);
	else
		return usrp()->_write_fpga_reg(FR_OE_3, (mask << 16) | value);
}

bool
rfic::rx_write_io(int value,int mask){

	if (int(d_which) == 0)
		return usrp()->_write_fpga_reg(FR_IO_1, (mask << 16) | value);
	else
		return usrp()->_write_fpga_reg(FR_IO_3, (mask << 16) | value);

}

bool
rfic::rx_set_atr_mask(int value){

	if (int(d_which) == 0)
		return usrp()->_write_fpga_reg(FR_ATR_MASK_1, value);
	else
		return usrp()->_write_fpga_reg(FR_ATR_MASK_3, value);

}

bool
rfic::rx_set_atr_txval(int value){

	if (int(d_which) == 0)
		return usrp()->_write_fpga_reg(FR_ATR_TXVAL_1, value);
	else
		return usrp()->_write_fpga_reg(FR_ATR_TXVAL_3, value);
}

bool
rfic::rx_set_atr_rxval(int value){

	if (int(d_which) == 0)
		return usrp()->_write_fpga_reg(FR_ATR_RXVAL_1, value);
	else
		return usrp()->_write_fpga_reg(FR_ATR_RXVAL_3, value);

}






void
rfic::send_reg(int reg_no,int dat )
{
    std::cout<<"value to be written on reg no "<<reg_no<<"is "<<dat<<std::endl;
    // dat is the data to write to register 0
    //spi_enable is the spi enables
    //spi_format is the spi format with a two-byte header
    //spi_format_no_header is the spi format, with no header
    //Send 16 bit header over SPI to send register number
    //Write 8 bit register
    //Set header
    int hdr = int((reg_no << 1) & 0x7fff);
    std::string abc = "105";
    //Set byte of write data
    //char c = (char)((dat & 0xff));
    //char * d = &c;

    bool check;
    std::cout<<"\nValue on the register before writing\n"<<get_reg(reg_no)<<std::endl;
    check = usrp()->_write_spi(hdr, d_spi_enable, d_spi_format, abc);
    if (check)
	std::cout<<"success"<<std::endl;
    else
	std::cout<<"fail"<<std::endl;
    std::cout<<"\nValue on the register after ******** writing\n ***"<<get_reg(reg_no)<<"***"<<std::endl;
    
}

char
rfic::get_reg(int reg_num)
{

	std::cout<<"inside get reg and register to be read is "<<reg_num<<std::endl;
	//Returns a vector containing the information in the first 320 registers in integer form
    //dat is the data to write to register 0
    //spi_enable is the spi enables
    //spi_format is the spi format with a two-byte header
    //spi_format_no_header is the spi format, with no header
    //u is the instance of the USRP
    int hdr = 0;
    int dat = 0;
    //Set byte of write data
    //char c = (char)((dat & 0xff));
    //char * d = &c;
    std::string d  = "0";
    bool check;
    check = usrp()->_write_spi(hdr, d_spi_enable, d_spi_format, d);
    
    std::string r; //string to be read from the register
    r = usrp()->_read_spi(0, d_spi_enable, d_spi_format_no_header, 64);
    r  = r + usrp()->_read_spi(0, d_spi_enable, d_spi_format_no_header, 64);
    r  = r + usrp()->_read_spi(0, d_spi_enable, d_spi_format_no_header, 64);
    r  = r + usrp()->_read_spi(0, d_spi_enable, d_spi_format_no_header, 64);
    r  = r + usrp()->_read_spi(0, d_spi_enable, d_spi_format_no_header, 64);
    std::cout<<"the contents of all registers are\n***"<<r<<"*****"<<std::endl;
    char read_val = (char)r[reg_num];
    return read_val;
}

void rfic  ::  set_reg_205(){	   
	int reg_205 = (   
	rx_lna << 5 |
	rx_lnab << 3 |
	rx_rxchen << 2 |
	rx_bbq_div2or4 << 1 |
	rx_Loselect << 0 );
	send_reg(205, reg_205);  
}

void rfic  ::  set_reg_0(){	
	int reg_0 = (   
	Ngt3 << 7 |
	NorNdiv4 << 0 );
	send_reg(0, reg_0);  
}
 
void rfic  ::  set_reg_1(){	   
	int reg_1 = (   
	RorFrNpRdiv4_25to18 << 0 );
	send_reg(1, reg_1);
}


void rfic  ::  set_reg_2(){	   
	int reg_2 = (   
	RorFrNpRdiv4_17to10 << 0 );
	send_reg(2, reg_2);
}

void rfic  ::  set_reg_3(){	   
	int reg_3 = (   
	RorFrNpRdiv4_9to2 << 0 );
	send_reg(3, reg_3);   
}


void rfic  ::  set_reg_4(){	   
	int reg_4 = (   
	RorFrNpRdiv4_1to0 << 6 );
	send_reg(4, reg_4);   
}
void rfic  ::  set_reg_5(){	   
	int reg_5 = (   
	Qu_tx_Ngt3 << 7 |
	NorNdiv4_phsh << 0 );
	send_reg(5, reg_5);
}
   
void rfic  ::  set_reg_6(){	   
	int reg_6 = (   
	RorFrNpRdiv4_phsh_25to18 << 0 );
	send_reg(6, reg_6);
}
   
void rfic  ::  set_reg_7(){	   
	int reg_7 = (   
	RorFrNpRdiv4_phsh_17to10 << 0 );
	send_reg(7, reg_7);
}
   
void rfic  ::  set_reg_8(){	   
	int reg_8 = (   
	RorFrNpRdiv4_phsh_9to2 << 0 );
	send_reg(8, reg_8);
}
   
void rfic  ::  set_reg_9(){	   
	int reg_9 = (   
	RorFrNpRdiv4_phsh_1to0 << 6 );
	send_reg(9, reg_9);
}
   
void rfic  ::  set_reg_10(){	   
	int reg_10 = (   
	Passthru_ref_clk << 7 |
	Byp_ram << 6 |
	Dis_adr_dith << 5 |
	Dis_p5G_dith << 4 |
	Byp_fine << 3 |
	Exclude32 << 2 |
	Dis_risedge << 1 |
	Dis_faledge << 0 );
	send_reg(10, reg_10);
}

   
void rfic  ::  set_reg_12(){	   
	int reg_12 = (   
	Spr_puls_en << 7 |
	Spr_puls_val_a_9to3 << 0 );
	send_reg(12, reg_12);
}
   
void rfic  ::  set_reg_13(){	   
	int reg_13 = (   
	Spr_pulse_val_2to0 << 5 );
	send_reg(13, reg_13);
}
   
void rfic  ::  set_reg_14(){	   
	int reg_14 = (   
	Spr_puls_val_b_9to2 << 0 );
	send_reg(14, reg_14);
}
   
void rfic  ::  set_reg_15(){	   
	int reg_15 = (   
	Spr_puls_val_b_1to0 << 6 );
	send_reg(15, reg_15);
}

   
void rfic  ::  set_reg_16(){	   
	int reg_16 = (   
	Thru_ris_en << 7 |
	Thru_ris_tap_11to6 << 1 );
	send_reg(16, reg_16);
}
   
void rfic  ::  set_reg_17(){	   
	int reg_17 = (   
	Thru_ris_tap_5to0 << 2 );
	send_reg(17, reg_17);
}
   
void rfic  ::  set_reg_18(){	   
	int reg_18 = (   
	Thru_fal_en << 7 |
	Thru_fal_tap_11to6 << 1 );
	send_reg(18, reg_18);
}   
void rfic  ::  set_reg_19(){	   
	int reg_19 = (   
	Thru_fal_tap_5to0 << 2 );
	send_reg(19, reg_19);
}   
void rfic  ::  set_reg_20(){	   
	int reg_20 = (   
	Dig_delay << 7 |
	Clk_driver_en << 6 |
	qu_reg_en << 5 |
	qq_reg_en << 4 |
	win_rst << 3 |
	fineEn << 2 |
	fineEnb << 1 |
	rsffEn << 0 );
	send_reg(20, reg_20);
}
   
void rfic  ::  set_reg_21(){	   
	int reg_21 = (   
	dl_en << 7 |
	cp_en << 6 |
	forceCpUpb << 5 |
	forceCpDn << 4 |
	pdUpTune_1to0 << 2 |
	pdDnTune_1to0 << 0 );
	send_reg(21, reg_21);   
}

void rfic  ::  set_reg_22(){	   
	int reg_22 = (   
	cpUpTune_2to0 << 5 |
	cpDnTune_2to0 << 2 |
	pdEn << 1 );
	send_reg(22, reg_22);
}
   
void rfic  ::  set_reg_23(){	   
	int reg_23 = (   
	digClkPhase_7to0 << 0 );
	send_reg(23, reg_23);
}   
void rfic  ::  set_reg_24(){	   
	int reg_24 = (   
	Rst_n_async << 7 );
	send_reg(24, reg_24); 
}  
void rfic  ::  read_reg_25(){	   
	int reg_25 = get_reg(25);   
	L1_lup00_15to8 = reg_25;   
}	   
void rfic  ::  read_reg_26(){	   
	int reg_26 = get_reg(26);   
	L1_lup90_15to8 = reg_26;   
}	   
void rfic  ::  read_reg_27(){	   
	int reg_27 = get_reg(27);   
	Merg_ris_fin = reg_27 >> 2;   
}	   
void rfic  ::  read_reg_28(){	   
	int reg_28 = get_reg(28);   
	Merg_fal_fin = reg_28 >> 2;   
}	   
void rfic  ::  set_reg_29(){	   
	int reg_29 = (   
	Qg00degDelay_0to4 << 3 );
	send_reg(29, reg_29);
}   
void rfic  ::  set_reg_30(){	   
	int reg_30 = (   
	Qg90degDelay_0to4 << 3 );
	send_reg(30, reg_30);
}   
void rfic  ::  set_reg_31(){	   
	int reg_31 = (   
	Qg180degDelay_0to4 << 3 );
	send_reg(31, reg_31);
}   
void rfic  ::  set_reg_32(){	   
	int reg_32 = (   
	Qg270degDelay_0to4 << 3 );
	send_reg(32, reg_32);
}   
void rfic  ::  set_reg_33(){	   
	int reg_33 = (   
	DischargeTap16_0to3 << 4 |
	ChargeTap16_0to3 << 0 );
	send_reg(33, reg_33);
}   
void rfic  ::  set_reg_34(){	   
	int reg_34 = (   
	DischargeTapn_0to3 << 4 |
	ChargeTapn16_0to3 << 0 );
	send_reg(34, reg_34);
}   
void rfic  ::  set_reg_35(){	   
	int reg_35 = (   
	X1sel_32to39 << 0 );
	send_reg(35, reg_35);   
}
void rfic  ::  set_reg_36(){	   
	int reg_36 = (   
	X1sel_40to47 << 0 );
	send_reg(36, reg_36);   
}
void rfic  ::  set_reg_37(){	   
	int reg_37 = (   
	X2sel_32to36 << 3 );
	send_reg(37, reg_37);   
}
void rfic  ::  set_reg_38(){	   
	int reg_38 = (   
	X2sel_37to41 << 3 );
	send_reg(38, reg_38);   
}
void rfic  ::  set_reg_39(){	   
	int reg_39 = (   
	X4sel_32to36 << 3 );
	send_reg(39, reg_39);   
}
void rfic  ::  set_reg_40(){	   
	int reg_40 = (   
	X4sel_37to41 << 3 );
	send_reg(40, reg_40);   
}
void rfic  ::  set_reg_41(){	   
	int reg_41 = (   
	X8sel_32to36 << 3 );
	send_reg(41, reg_41);   
}
void rfic  ::  set_reg_42(){	   
	int reg_42 = (   
	X8sel_41 << 7 |
	X8sel_37to40 << 3 );
	send_reg(42, reg_42);   
}
void rfic  ::  set_reg_43(){	   
	int reg_43 = (   
	qutx_fwd_180Cal_en << 7 |
	qutx_fwd_0Cal_en << 6 );
	send_reg(43, reg_43);   
}
void rfic  ::  set_reg_48(){	   
	int reg_48 = (   
	Ngt3_2 << 7 |
	NorNdiv4_2 << 0 );
	send_reg(48, reg_48);   
}
void rfic  ::  set_reg_49(){	   
	int reg_49 = (   
	RorFrNpRdiv4_25to18_2 << 0 );
	send_reg(49, reg_49);   
}
void rfic  ::  set_reg_50(){	   
	int reg_50 = (   
	RorFrNpRdiv4_17to10_2 << 0 );
	send_reg(50, reg_50);   
}
void rfic  ::  set_reg_51(){	   
	int reg_51 = (   
	RorFrNpRdiv4_9to2_2 << 0 );
	send_reg(51, reg_51);   
}
void rfic  ::  set_reg_52(){	   
	int reg_52 = (   
	RorFrNpRdiv4_1to0_2 << 6 );
	send_reg(52, reg_52);   
}
void rfic  ::  set_reg_53(){	   
	int reg_53 = (   
	Qu_tx_Ngt3_2 << 7 |
	NorNdiv4_phsh_2 << 0 );
	send_reg(52, reg_53);   
}
void rfic  ::  set_reg_54(){	   
	int reg_54 = (   
	RorFrNpRdiv4_phsh_25to18_2 << 0 );
	send_reg(54, reg_54);   
}
void rfic  ::  set_reg_55(){	   
	int reg_55 = (   
	RorFrNpRdiv4_phsh_17to10_2 << 0 );
	send_reg(55, reg_55);   
}
void rfic  ::  set_reg_56(){	   
	int reg_56 = (   
	RorFrNpRdiv4_phsh_9to2_2 << 0 );
	send_reg(56, reg_56);   
}
void rfic  ::  set_reg_57(){	   
	int reg_57 = (   
	RorFrNpRdiv4_phsh_1to0_2 << 6 );
	send_reg(57, reg_57);   
}
void rfic  ::  set_reg_58(){	   
	int reg_58 = (   
	Passthru_ref_clk_2 << 7 |
	Byp_ram_2 << 6 |
	Dis_adr_dith_2 << 5 |
	Dis_p5G_dith_2 << 4 |
	Byp_fine_2 << 3 |
	Exclude32_2 << 2 |
	Dis_risedge_2 << 1 |
	Dis_faledge_2 << 0 );
	send_reg(58, reg_58);   
}
void rfic  ::  set_reg_60(){	   
	int reg_60 = (   
	Spr_puls_en_2 << 7 |
	Spr_puls_val_a_9to3_2 << 0 );
	send_reg(60, reg_60);   
}
void rfic  ::  set_reg_61(){	   
	int reg_61 = (   
	Spr_pulse_val_2to0_2 << 5 );
	send_reg(61, reg_61);   
}
void rfic  ::  set_reg_62(){	   
	int reg_62 = (   
	Spr_puls_val_b_9to2_2 << 0 );
	send_reg(62, reg_62);   
}
void rfic  ::  set_reg_63(){	   
	int reg_63 = (   
	Spr_puls_val_b_1to0_2 << 6 );
	send_reg(63, reg_63);   
}
void rfic  ::  set_reg_64(){	   
	int reg_64 = (   
	Thru_ris_en_2 << 7 |
	Thru_ris_tap_11to6_2 << 1 );
	send_reg(64, reg_64);   
}
void rfic  ::  set_reg_65(){	   
	int reg_65 = (   
	Thru_ris_tap_5to0_2 << 2 );
	send_reg(65, reg_65);   
}
void rfic  ::  set_reg_66(){	   
	int reg_66 = (   
	Thru_fal_en_2 << 7 |
	Thru_fal_tap_11to6_2 << 1 );
	send_reg(66, reg_66);   
}
void rfic  ::  set_reg_67(){	   
	int reg_67 = (   
	Thru_fal_tap_5to0_2 << 2 );
	send_reg(67, reg_67);   
}
void rfic  ::  set_reg_68(){	   
	int reg_68 = (   
	Dig_delay_2 << 7 |
	Clk_driver_en_2 << 6 |
	qu_reg_en_2 << 5 |
	qq_reg_en_2 << 4 |
	win_rst_2 << 3 |
	fineEn_2 << 2 |
	fineEnb_2 << 1 |
	rsffEn_2 << 0 );
	send_reg(68, reg_68);   
}
void rfic  ::  set_reg_69(){	   
	int reg_69 = (   
	dl_en_2 << 7 |
	cp_en_2 << 6 |
	forceCpUpb_2 << 5 |
	forceCpDn_2 << 4 |
	pdUpTune_1to0_2 << 2 |
	pdDnTune_1to0_2 << 0 );
	send_reg(69, reg_69);   
}
void rfic  ::  set_reg_70(){	   
	int reg_70 = (   
	cpUpTune_2to0_2 << 5 |
	cpDnTune_2to0_2 << 2 |
	pdEn_2 << 1 );
	send_reg(70, reg_70);   
}
void rfic  ::  set_reg_71(){	   
	int reg_71 = (   
	digClkPhase_7to0_2 << 0 );
	send_reg(71, reg_71);   
}
void rfic  ::  set_reg_72(){	   
	int reg_72 = (   
	Rst_n_async_2 << 7 );
	send_reg(72, reg_72);   
}
void rfic  ::  read_reg_73(){	   
	int reg_73 = get_reg(73);   
	L1_lup00_15to8_2 = reg_73;  
}	   
void rfic  ::  read_reg_74(){	   
	int reg_74 = get_reg(74);   
	L1_lup90_15to8_2 = reg_74;
}	   
void rfic  ::  read_reg_75(){	   
	int reg_75 = get_reg(75);   
	Merg_ris_fin_2 = reg_75 >> 2;
}	   
void rfic  ::  read_reg_76(){	   
	int reg_76 = get_reg(76);   
	Merg_fal_fin_2 = reg_76 >> 2;   
}	   
void rfic  ::  set_reg_77(){	   
	int reg_77 = (   
	Qg00degDelay_0to4_2 << 3 );
	send_reg(77, reg_77);   
}
void rfic  ::  set_reg_78(){	   
	int reg_78 = (   
	Qg90degDelay_0to4_2 << 3 );
	send_reg(78, reg_78);   
}
void rfic  ::  set_reg_79(){	   
	int reg_79 = (   
	Qg180degDelay_0to4_2 << 3 );
	send_reg(79, reg_79);   
}
void rfic  ::  set_reg_80(){	   
	int reg_80 = (   
	Qg270degDelay_0to4_2 << 3 );
	send_reg(80, reg_80);   
}
void rfic  ::  set_reg_81(){	   
	int reg_81 = (   
	DischargeTap16_3to0 << 4 |
	ChargeTap16_3to0 << 0 );
	send_reg(81, reg_81);   
}
void rfic  ::  set_reg_82(){	   
	int reg_82 = (   
	DischargeTapn_3to0 << 4 |
	ChargeTapn16_3to0 << 0 );
	send_reg(82, reg_82);   
}
void rfic  ::  set_reg_83(){	   
	int reg_83 = (   
	X1sel_32to39_2 << 0 );
	send_reg(83, reg_83);   
}
void rfic  ::  set_reg_84(){	   
	int reg_84 = (   
	X1sel_40to47_2 << 0 );
	send_reg(84, reg_84);   
}
void rfic  ::  set_reg_85(){	   
	int reg_85 = (   
	X2sel_32to36_2 << 3 );
	send_reg(85, reg_85);   
}
void rfic  ::  set_reg_86(){	   
	int reg_86 = (   
	X2sel_37to41_2 << 3 );
	send_reg(86, reg_86);   
}
void rfic  ::  set_reg_87(){	   
	int reg_87 = (   
	X4sel_32to36_2 << 3 );
	send_reg(87, reg_87);   
}
void rfic  ::  set_reg_88(){	   
	int reg_88 = (   
	X4sel_37to41_2 << 3 );
	send_reg(88, reg_88);   
}
void rfic  ::  set_reg_89(){	   
	int reg_89 = (   
	X8sel_32to36_2 << 3 );
	send_reg(89, reg_89);   
}
void rfic  ::  set_reg_90(){	   
	int reg_90 = (   
	X8sel_41_2 << 7 |
	X8sel_37to40_2 << 3 );
	send_reg(90, reg_90);   
}
void rfic  ::  set_reg_91(){	   
	int reg_91 = (   
	qutx_fb_180Cal_en << 7 |
	qutx_fb_0Cal_en << 6 |
	qutx_fb_180Rsff_en << 5 |
	qutx_fb_0Rsff_en << 4 );
	send_reg(91, reg_91);   
}
void rfic  ::  set_reg_96(){	   
	int reg_96 = (   
	N << 4 |
	R_11to8 << 0 );
	send_reg(96, reg_96);   
}
void rfic  ::  set_reg_97(){	   
	int reg_97 = (   
	R_7to0 << 0 );
	send_reg(97, reg_97);   
}
void rfic  ::  set_reg_98(){	   
	int reg_98 = (   
	Asyncrst_n << 7 |
	Cp_sel_6to0 << 0 );
	send_reg(98, reg_98);   
}
void rfic  ::  set_reg_99(){	   
	int reg_99 = (   
	Cp_sel_8to7 << 6 |
	ForceFout << 5 |
	ForceFoutb << 4 |
	Out_en << 3 |
	Dll_en << 2 |
	Ana_en << 1 );
	send_reg(99, reg_99);   
}
void rfic  ::  read_reg_100(){	   
	int reg_100 = get_reg(100);   
	Decod_in_0deg = reg_100 >> 3;
}	   
void rfic  ::  set_reg_104(){	   
	int reg_104 = (   
	Ngt3_3 << 7 |
	NorNdiv4_3 << 0 );
	send_reg(104, reg_104);   
}
void rfic  ::  set_reg_105(){	   
	int reg_105 = (   
	RorFrNpRdiv4_25to18_3 << 0 );
	send_reg(105, reg_105);   
}
void rfic  ::  set_reg_106(){	   
	int reg_106 = (   
	RorFrNpRdiv4_17to10_3 << 0 );
	send_reg(106, reg_106);   
}
void rfic  ::  set_reg_107(){	   
	int reg_107 = (   
	RorFrNpRdiv4_9to2_3 << 0 );
	send_reg(107, reg_107);   
}
void rfic  ::  set_reg_108(){	   
	int reg_108 = (   
	RorFrNpRdiv4_1to0_3 << 6 );
	send_reg(108, reg_108);   
}
void rfic  ::  set_reg_109(){	   
	int reg_109 = (   
	Qu_tx_Ngt3_3 << 7 |
	NorNdiv4_phsh_3 << 0 );
	send_reg(109, reg_109);   
}
void rfic  ::  set_reg_110(){	   
	int reg_110 = (   
	RorFrNpRdiv4_phsh_25to18_3 << 0 );
	send_reg(110, reg_110);   
}
void rfic  ::  set_reg_111(){	   
	int reg_111 = (   
	RorFrNpRdiv4_phsh_17to10_3 << 0 );
	send_reg(111, reg_111);   
}
void rfic  ::  set_reg_112(){	   
	int reg_112 = (   
	RorFrNpRdiv4_phsh_9to2_3 << 0 );
	send_reg(112, reg_112);   
}
void rfic  ::  set_reg_113(){	   
	int reg_113 = (   
	RorFrNpRdiv4_phsh_1to0_3 << 6 );
	send_reg(113, reg_113);   
}
void rfic  ::  set_reg_114(){	   
	int reg_114 = (   
	Passthru_ref_clk_3 << 7 |
	Byp_ram_3 << 6 |
	Dis_adr_dith_3 << 5 |
	Dis_p5G_dith_3 << 4 |
	Byp_fine_3 << 3 |
	Exclude32_3 << 2 |
	Dis_risedge_3 << 1 |
	Dis_faledge_3 << 0 );
	send_reg(114, reg_114);   
}
void rfic  ::  set_reg_116(){	   
	int reg_116 = (   
	Spr_puls_en_3 << 7 |
	Spr_puls_val_a_9to3_3 << 0 );
	send_reg(116, reg_116);   
}
void rfic  ::  set_reg_117(){	   
	int reg_117 = (   
	Spr_pulse_val_2to0_3 << 5 );
	send_reg(117, reg_117);   
}
void rfic  ::  set_reg_118(){	   
	int reg_118 = (   
	Spr_puls_val_b_9to2_3 << 0 );
	send_reg(118, reg_118);   
}
void rfic  ::  set_reg_119(){	   
	int reg_119 = (   
	Spr_puls_val_b_1to0_3 << 6 );
	send_reg(119, reg_119);   
}
void rfic  ::  set_reg_120(){	   
	int reg_120 = (   
	Thru_ris_en_3 << 7 |
	Thru_ris_tap_11to6_3 << 1 );
	send_reg(120, reg_120);   
}
void rfic  ::  set_reg_121(){	   
	int reg_121 = (   
	Thru_ris_tap_5to0_3 << 2 );
	send_reg(121, reg_121);   
}
void rfic  ::  set_reg_122(){	   
	int reg_122 = (   
	Thru_fal_en_3 << 7 |
	Thru_fal_tap_11to6_3 << 1 );
	send_reg(122, reg_122);   
}
void rfic  ::  set_reg_123(){	   
	int reg_123 = (   
	Thru_fal_tap_5to0_3 << 2 );
	send_reg(123, reg_123);   
}
void rfic  ::  set_reg_124(){	   
	int reg_124 = (   
	Dig_delay_3 << 7 |
	Clk_driver_en_3 << 6 |
	qu_reg_en_3 << 5 |
	qq_reg_en_3 << 4 |
	win_rst_3 << 3 |
	fineEn_3 << 2 |
	fineEnb_3 << 1 |
	rsffEn_3 << 0 );
	send_reg(124, reg_124);   
}
void rfic  ::  set_reg_125(){	   
	int reg_125 = (   
	dl_en_3 << 7 |
	cp_en_3 << 6 |
	forceCpUpb_3 << 5 |
	forceCpDn_3 << 4 |
	pdUpTune_1to0_3 << 2 |
	pdDnTune_1to0_3 << 0 );
	send_reg(125, reg_125);   
}
void rfic  ::  set_reg_126(){	   
	int reg_126 = (   
	cpUpTune_2to0_3 << 5 |
	cpDnTune_2to0_3 << 2 |
	pdEn_3 << 1 );
	send_reg(126, reg_126);   
}
void rfic  ::  set_reg_127(){	   
	int reg_127 = (   
	digClkPhase_7to0_3 << 0 );
	send_reg(127, reg_127);   
}
void rfic  ::  set_reg_128(){	   
	int reg_128 = (   
	Rst_n_async_3 << 7 );
	send_reg(128, reg_128);   
}
void rfic  ::  read_reg_129(){	   
	int reg_129 = get_reg(129);   
	L1_lup00_15to8_3 = reg_129;
}	   
void rfic  ::  read_reg_130(){	   
	int reg_130 = get_reg(130);   
	L1_lup90_15to8_3 = reg_130;   
}	   
void rfic  ::  read_reg_131(){	   
	int reg_131 = get_reg(131);   
	Merg_ris_fin_3 = reg_131 >> 2;   
}	   
void rfic  ::  read_reg_132(){	   
	int reg_132 = get_reg(132);   
	Merg_fal_fin_3 = reg_132 >> 2;   
}	   
void rfic  ::  set_reg_133(){	   
	int reg_133 = (   
	Qg00degDelay_0to4_3 << 3 );
	send_reg(133, reg_133);   
}
void rfic  ::  set_reg_134(){	   
	int reg_134 = (   
	Qg90degDelay_0to4_3 << 3 );
	send_reg(134, reg_134);   
}
void rfic  ::  set_reg_135(){	   
	int reg_135 = (   
	Qg180degDelay_0to4_3 << 3 );
	send_reg(135, reg_135);   
}
void rfic  ::  set_reg_136(){	   
	int reg_136 = (   
	Qg270degDelay_0to4_3 << 3 );
	send_reg(136, reg_136);   
}
void rfic  ::  set_reg_137(){	   
	int reg_137 = (   
	DischargeTap16_0to3_3 << 4 |
	ChargeTap16_0to3_3 << 0 );
	send_reg(137, reg_137);   
}
void rfic  ::  set_reg_138(){	   
	int reg_138 = (   
	DischargeTapn_0to3_3 << 4 |
	ChargeTapn16_0to3_3 << 0 );
	send_reg(138, reg_138);   
}
void rfic  ::  set_reg_139(){	   
	int reg_139 = (   
	X1sel_32to39_3 << 0 );
	send_reg(139, reg_139);   
}
void rfic  ::  set_reg_140(){	   
	int reg_140 = (   
	X1sel_40to47_3 << 0 );
	send_reg(140, reg_140);   
}
void rfic  ::  set_reg_141(){	   
	int reg_141 = (   
	X2sel_32to36_3 << 3 );
	send_reg(141, reg_141);   
}
void rfic  ::  set_reg_142(){	   
	int reg_142 = (   
	X2sel_37to41_3 << 3 );
	send_reg(142, reg_142);   
}
void rfic  ::  set_reg_143(){	   
	int reg_143 = (   
	X4sel_32to36_3 << 3 );
	send_reg(143, reg_143);   
}
void rfic  ::  set_reg_144(){	   
	int reg_144 = (   
	X4sel_37to41_3 << 3 );
	send_reg(144, reg_144);   
}
void rfic  ::  set_reg_145(){	   
	int reg_145 = (   
	X8sel_32to36_3 << 3 );
	send_reg(145, reg_145);   
}
void rfic  ::  set_reg_146(){	   
	int reg_146 = (   
	X8sel_41_3 << 7 |
	X8sel_37to40_3 << 3 );
	send_reg(146, reg_146);   
}
void rfic  ::  set_reg_147(){	   
	int reg_147 = (   
	qurx_180Cal_en << 7 |
	qurx_0Cal_en << 6 );
	send_reg(147, reg_147);   
}
void rfic  ::  set_reg_152(){	   
	int reg_152 = (   
	extClkEn << 7 |
	extClkEnBNOTD7 << 6 |
	div2_rst << 5 |
	TxChClkSel << 3 );
	send_reg(152, reg_152);   
}
void rfic  ::  set_reg_153(){	   
	int reg_153 = (   
	TxChClkEn << 5 );
	send_reg(153, reg_153);   
}
void rfic  ::  set_reg_156(){	   
	int reg_156 = (   
	tx_bb_en << 7 |
	tx_bb_fdbk_bw << 5 |
	tx_bb_fdbk_cal_en << 4 |
	tx_bb_fdbk_cart_err_en << 3 |
	tx_bb_fdbk_cart_fb_en << 2 |
	tx_bb_fdbk_cart_fwd_en << 1 );
	send_reg(156, reg_156);   
}
void rfic  ::  set_reg_157(){	   
	int reg_157 = (   
	tx_bb_fdbk_en << 6 |
	tx_bb_fdbk_1q_sel << 5 |
	tx_bb_fdbk_lp << 2 );
	send_reg(157, reg_157);   
}
void rfic  ::  set_reg_158(){	   
	int reg_158 = (   
	tx_bb_fdbk_statt << 5 |
	tx_bb_fdbk_swapi << 4 |
	tx_bb_fdbk_swapq << 3 |
	tx_bb_gain_cmp << 2 );
	send_reg(158, reg_158);   
}
void rfic  ::  set_reg_159(){	   
	int reg_159 = (   
	tx_bb_lp << 5 |
	tx_bb_swapi << 4 |
	tx_bb_swapq << 3 |
	tx_butt_bw << 0 );
	send_reg(159, reg_159);   
}
void rfic  ::  set_reg_160(){	   
	int reg_160 = (   
	tx_bw_trck << 4 |
	tx_cart_en << 3 );
	send_reg(160, reg_160);   
}
void rfic  ::  set_reg_161(){	   
	int reg_161 = (   
	tx_cart_fb_bb_statt << 3 );
	send_reg(161, reg_161);   
}
void rfic  ::  set_reg_162(){	   
	int reg_162 = (   
	tx_cart_fb_dcoc_dac_I1 << 2 );
	send_reg(162, reg_162);
}   
void rfic  ::  set_reg_163(){	   
	int reg_163 = (   
	tx_cart_fb_dcoc_dac_I2 << 2 );
	send_reg(163, reg_163);   
}
void rfic  ::  set_reg_164(){	   
	int reg_164 = (   
	tx_cart_fb_dcoc_dac_Q1 << 2 );
	send_reg(164, reg_164);   
}
void rfic  ::  set_reg_165(){	   
	int reg_165 = (   
	tx_cart_fb_dcoc_dac_Q2 << 2 );
	send_reg(165, reg_165);   
}
void rfic  ::  set_reg_166(){	   
	int reg_166 = (   
	CartesianFeedbackpathDCOCenable << 7 |
	CartesianFeedbackpathenable << 6 |
	CartesianFBpathHiResolutionDCOCenable << 5 |
	CartesianFBpathBW << 1 );
	send_reg(166, reg_166);   
}
void rfic  ::  set_reg_167(){	   
	int reg_167 = (   
	CartesianFBRFGain << 2 );
	send_reg(167, reg_167);   
}
void rfic  ::  set_reg_168(){	   
	int reg_168 = (   
	CartesianFBpathSwapIandIx << 7 |
	CartesianFBpathSwapQandQx << 6 |
	CartesianFBpathSwitchtoforwardSummer << 5 |
	tx_cart_fb_lo_select << 0 );
	send_reg(168, reg_168);   
}
void rfic  ::  set_reg_169(){	   
	int reg_169 = (   
	CartesianFBpathAmp1Gain << 6 |
	CartesianFBpathAmp2Gain << 4 |
	CartesianFBpathAmp3Gain << 2 |
	CartesianFBpathAmp4Gain << 0 );
	send_reg(169, reg_169);   
}
void rfic  ::  set_reg_170(){	   
	int reg_170 = (   
	CartesianFBpathAmpCurrentSelect << 5 |
	CartesianFBpathZeroEnable << 4 |
	tx_cart_zero_statt << 0 );
	send_reg(170, reg_170);   
}
void rfic  ::  set_reg_171(){	   
	int reg_171 = (   
	tx_inbuf_bw << 6 |
	tx_inbuf_statt << 3 );
	send_reg(171, reg_171);   
}
void rfic  ::  set_reg_172(){	   
	int reg_172 = (   
	tx_output_channel_sel << 5 );
	send_reg(172, reg_172);   
}
void rfic  ::  set_reg_173(){	   
	int reg_173 = (   
	tx_p1_bw << 4 |
	tx_pw_bw1 << 2 );
	send_reg(173, reg_173);   
}
void rfic  ::  set_reg_174(){	   
	int reg_174 = (   
	tx_p2_bw2 << 4 |
	PushPullBufferCurrent << 1 );
	send_reg(174, reg_174);   
}
void rfic  ::  set_reg_175(){	   
	int reg_175 = (   
	tx_rf_aoc_bw << 6 |
	RFForwardPathEnable_toMUX << 5 |
	RFForwardPathEnable_ExternalPinenable << 4 |
	tx_rf_fwd_lp << 1 );
	send_reg(175, reg_175);   
}
void rfic  ::  set_reg_176(){	   
	int reg_176 = (   
	tx_rf_fwd_statt1 << 5 |
	tx_rf_fwd_statt2 << 2 );
	send_reg(176, reg_176);   
}
void rfic  ::  set_reg_177(){	   
	int reg_177 = (   
	BBQDivideby2or4Select << 7 |
	BBQQuadGenEnable << 6 |
	BBQPolyphaseQuadGenEnable << 5 );
	send_reg(177, reg_177);   
}
void rfic  ::  set_reg_178(){	   
	int reg_178 = (   
	lofb_tun_s << 4 |
	lofb_tun_sx << 0 );
	send_reg(178, reg_178);   
}
void rfic  ::  set_reg_179(){	   
	int reg_179 = (   
	lofw_tun_s2 << 4 |
	lofw_tun_sx2 << 0 );
	send_reg(179, reg_179);   
}
void rfic  ::  set_reg_180(){	   
	int reg_180 = (   
	reserve_tx26 << 0 );
	send_reg(180, reg_180);   
}
void rfic  ::  set_reg_181(){	   
	int reg_181 = (   
	reserve_tx27 << 0 );
	send_reg(181, reg_181);   
}
void rfic  ::  set_reg_192(){	   
	int reg_192 = (   
	rx_Idac << 3 |
	rx_dcs << 1 |
	rx_den << 0 );
	send_reg(192, reg_192);   
}
void rfic  ::  set_reg_193(){	   
	int reg_193 = (   
	rx_Qdac << 3 |
	rx_cmpen << 1 |
	rx_dcoc << 0 );
	send_reg(193, reg_193);   
}
void rfic  ::  set_reg_194(){	   
	int reg_194 = (   
	rx_ten << 7 |
	rx_ren << 6 |
	rx_dven << 4 |
	rx_dv << 0 );
	send_reg(194, reg_194);   
}
void rfic  ::  set_reg_195(){	   
	int reg_195 = (   
	rx_extc << 7 |
	rx_cen << 4 |
	rx_chck << 2 |
	rx_chcken << 1 |
	rx_fen << 0 );
	send_reg(195, reg_195);   
}
void rfic  ::  set_reg_196(){	   
	int reg_196 = (   
	rx_onchen << 7 |
	rx_offchen << 6 |
	rx_foe << 0 );
	send_reg(196, reg_196);   
}
void rfic  ::  set_reg_197(){	   
	int reg_197 = (   
	rx_offch << 5 |
	rx_onchf << 3 |
	rx_onchc << 1 );
	send_reg(197, reg_197);   
}
void rfic  ::  set_reg_198(){	   
	int reg_198 = (   
	rx_qs << 5 |
	rx_bqg << 3 |
	rx_rq << 0 );
	send_reg(198, reg_198);   
}
void rfic  ::  set_reg_199(){	   
	int reg_199 = (   
	rx_rv << 5 |
	rx_rip << 2 |
	rx_rfp << 0 );
	send_reg(199, reg_199);   
}
void rfic  ::  set_reg_200(){	   
	int reg_200 = (   
	rx_cp_12to8 << 3 |
	rx_gs << 0 );
	send_reg(200, reg_200);   
}
void rfic  ::  set_reg_201(){	   
	int reg_201 = (   
	rx_cp_7to0 << 0 );
	send_reg(201, reg_201);   
}
void rfic  ::  set_reg_202(){	   
	int reg_202 = (   
	rx_cv_10to3 << 0 );
	send_reg(202, reg_202);   
}
void rfic  ::  set_reg_203(){	   
	int reg_203 = (   
	rx_cv_2to0 << 5 |
	rx_cc_2to0 << 2 |
	rx_cq_9to8 << 0 );
	send_reg(203, reg_203);   
}
void rfic  ::  set_reg_204(){	   
	int reg_204 = (   
	rx_cq_7to0 << 0 );
	send_reg(204, reg_204);   
}
 

void rfic  ::  set_reg_206(){	   
	int reg_206 = (   
	poly_en << 7 );
	send_reg(206, reg_206);   
}
void rfic  ::  set_reg_207(){	   
	int reg_207 = (   
	lorx_tun_s << 4 |
	lorx_tun_sx << 0 );
	send_reg(207, reg_207);   
}
void rfic  ::  read_reg_208(){	   
	int reg_208 = get_reg(208);   
	rx_Icmpo = reg_208 >> 5;   
	rx_Iodac = reg_208 % 64;   
}	   
void rfic  ::  read_reg_209(){	   
	int reg_209 = get_reg(209);   
	rx_Qcmpo = reg_209 >> 5;   
	rx_Qodac = reg_209 % 64;   
}	   
void rfic  ::  read_reg_210(){	   
	int reg_210 = get_reg(210);   
	rx_rc = reg_210;   
}	   
void rfic  ::  set_reg_220(){	   
	int reg_220 = (   
	shft_cml_in << 7 |
	vagenable1 << 6 |
	vagenable2 << 5 );
	send_reg(220, reg_220);   
}
void rfic  ::  set_reg_222(){	   
	int reg_222 = (   
	TestMuxBufferEnable << 7 |
	TestMuxEnable << 6 |
	TestMuxSetting << 0 );
	send_reg(222, reg_222);
}


bool rfic :: set_rx_gain(float gain){
	// Set RX gain
	// @param gain){ gain in dB
	// Four parameters){ rx_bqg, rx_dcs, rx_gs, rx_rip
	// 1 to 39 dB of gain (0 to 38){
	// Not all steps available
	if (gain < 0.0) gain = 0.0;
	if (gain > 38.0)gain = 38.0;

	if (gain <= 3){
		rx_bqg = 3;	//reg 198
		rx_dcs = 0;	//reg 192
		rx_gs = 4;	//reg 200
		rx_rip = 4;	//reg 199
	}
	else if ( gain >= 3 && gain < 4){
		rx_bqg = 3;
		rx_dcs = 0;
		rx_gs = 4;
		rx_rip = 3;
	}
	else if ( gain >= 4 && gain < 5){
		rx_bqg = 3;
		rx_dcs = 0;
		rx_gs = 2;
		rx_rip = 4;
	}
	else if ( gain >=5  && gain < 6){
		rx_bqg = 3;
		rx_dcs = 0;
		rx_gs = 3;
		rx_rip = 3;
	}
	else if ( gain >= 6 && gain < 7){
		rx_bqg = 3;
		rx_dcs = 0;
		rx_gs = 4;
		rx_rip = 2;
	}
	else if ( gain >= 7 && gain < 8){
		rx_bqg = 3;
		rx_dcs = 0;
		rx_gs = 2;
		rx_rip = 3;
	}
	else if ( gain >= 8 && gain < 9){
		rx_bqg = 3;
		rx_dcs = 0;
		rx_gs = 3;
		rx_rip = 2;
	}
	else if ( gain >= 9 && gain < 10){
		rx_bqg = 3;
		rx_dcs = 0;
		rx_gs = 1;
		rx_rip = 3;
	}
	else if ( gain >= 10 && gain < 11){
		rx_bqg = 3;
		rx_dcs = 0;
		rx_gs = 2;
		rx_rip = 2;
	}
	else if ( gain >= 11 && gain < 12){
		rx_bqg = 3;
		rx_dcs = 0;
		rx_gs = 0;
		rx_rip = 3;
	}
	else if ( gain >= 12 && gain < 13){
		rx_bqg = 2;
		rx_dcs = 0;
		rx_gs = 4;
		rx_rip = 2;
	}
	else if ( gain >= 13 && gain < 14){
		rx_bqg = 3;
		rx_dcs = 0;
		rx_gs = 2;
		rx_rip = 1;
	}
	else if ( gain >= 14 && gain < 15){
		rx_bqg = 3;
		rx_dcs = 0;
		rx_gs = 0;
		rx_rip = 2;
	}
	else if ( gain >= 15 && gain < 16){
		rx_bqg = 2;
		rx_dcs = 0;
		rx_gs = 1;
		rx_rip = 3;
	}
	else if ( gain >= 16 && gain < 17){
		rx_bqg = 2;
		rx_dcs = 0;
		rx_gs = 2;
		rx_rip = 2;
	}
	else if ( gain >= 17 && gain < 18){
		rx_bqg = 3;
		rx_dcs = 0;
		rx_gs = 0;
		rx_rip = 2;
	}
	else if ( gain >= 18 && gain < 19){
		rx_bqg = 3;
		rx_dcs = 0;
		rx_gs = 1;
		rx_rip = 0;
	}
	else if ( gain >= 19 && gain < 20){
		rx_bqg = 2;
		rx_dcs = 0;
		rx_gs = 2;
		rx_rip = 1;
	}
	else if ( gain >= 20 && gain < 21){
		rx_bqg = 3;
		rx_dcs = 0;
		rx_gs = 0;
		rx_rip = 0;
	}
	else if ( gain >= 21 && gain < 22){
		rx_bqg = 2;
		rx_dcs = 0;
		rx_gs = 1;
		rx_rip = 1;
	}
	else if ( gain >= 22 && gain < 23){
		rx_bqg = 1;
		rx_dcs = 1;
		rx_gs = 2;
		rx_rip = 2;
	}
	else if ( gain >= 23 && gain < 24){
		rx_bqg = 2;
		rx_dcs = 0;
		rx_gs = 0;
		rx_rip = 1;
	}
	else if ( gain >= 24 && gain < 25){
		rx_bqg = 1;
		rx_dcs = 1;
		rx_gs = 1;
		rx_rip = 2;
	}
	else if ( gain >= 25 && gain < 26){
		rx_bqg = 1;
		rx_dcs = 1;
		rx_gs = 2;
		rx_rip = 1;
	}
	else if ( gain >= 26 && gain < 27){
		rx_bqg = 1;
		rx_dcs = 1;
		rx_gs = 3;
		rx_rip = 0;
	}
	else if ( gain >= 27 && gain < 28){
		rx_bqg = 1;
		rx_dcs = 1;
		rx_gs = 1;
		rx_rip = 1;
	}
	else if ( gain >= 28 && gain < 29){
		rx_bqg = 1;
		rx_dcs = 1;
		rx_gs = 2;
		rx_rip = 0;
	}
	else if ( gain >= 29 && gain < 30){
		rx_bqg = 1;
		rx_dcs = 1;
		rx_gs = 0;
		rx_rip = 1;
	}
	else if ( gain >= 30 && gain < 31){
		rx_bqg = 1;
		rx_dcs = 1;
		rx_gs = 1;
		rx_rip = 0;
	}
	else if ( gain >= 31 && gain < 32){
		rx_bqg = 0;
		rx_dcs = 3;
		rx_gs = 2;
		rx_rip = 1;
	}
	else if ( gain >= 32 && gain < 33){
		rx_bqg = 1;
		rx_dcs = 1;
		rx_gs = 0;
		rx_rip = 0;
	}
	else if ( gain >= 33 && gain < 34){
		rx_bqg = 0;
		rx_dcs = 3;
		rx_gs = 1;
		rx_rip = 1;
	}
	else if ( gain >= 34 && gain < 35){
		rx_bqg = 0;
		rx_dcs = 3;
		rx_gs = 2;
		rx_rip = 0;
	}
	else if ( gain >= 35 && gain < 36){
		rx_bqg = 0;
		rx_dcs = 3;
		rx_gs = 0;
		rx_rip = 1;
	}
	else if ( gain >= 36 && gain < 38){
		rx_bqg = 0;
		rx_dcs = 3;
		rx_gs = 1;
		rx_rip = 0;
	}
	else if ( gain >= 38){
		rx_bqg = 0;
		rx_dcs = 3;
		rx_gs = 0;
		rx_rip = 0;
	}
	set_reg_198();
	set_reg_192();
	set_reg_200();
	set_reg_199();
	return true; //this function always returns true. Maximum gain is 38
}

bool rfic :: set_tx_gain(float gain){
	// Set TX gain
	// @param gain){ output gain in dB
	// Two parameters){
	// tx_rf_fwd_statt1, tx_rf_fwd_statt2
	// (45 dB of range)
	// 5 dB steps
	if (gain < 0.0) gain = 0.0;
	if (gain > 45.0) gain = 45.0;

	if (gain <= 2.5){
		tx_rf_fwd_statt1 = 7;
		tx_rf_fwd_statt2 = 7;
	}
	else if ( gain > 2.5 && gain <= 7.5){
		tx_rf_fwd_statt1 = 3;
		tx_rf_fwd_statt2 = 7;
	}
	else if ( gain > 7.5 && gain <= 12.5){
		tx_rf_fwd_statt1 = 1;
		tx_rf_fwd_statt2 = 7;
	}
	else if ( gain > 12.5 && gain <= 17.5){
		tx_rf_fwd_statt1 = 3;
		tx_rf_fwd_statt2 = 3;
	}
	else if ( gain > 17.5 && gain <= 22.5){
		tx_rf_fwd_statt1 = 1;
		tx_rf_fwd_statt2 = 3;
	}
	else if ( gain > 22.5 && gain <= 27.5){
		tx_rf_fwd_statt1 = 0;
		tx_rf_fwd_statt2 = 3;
	}		
	else if ( gain > 27.5 && gain <= 32.5){
		tx_rf_fwd_statt1 = 1;
		tx_rf_fwd_statt2 = 1;
	}
	else if ( gain > 32.5 && gain <= 37.5){
		tx_rf_fwd_statt1 = 0;
		tx_rf_fwd_statt2 = 1;
	}
	else if ( gain > 37.5 && gain <= 42.5){
		tx_rf_fwd_statt1 = 1;
		tx_rf_fwd_statt2 = 0;
	}
	else if ( gain > 42.5){
		tx_rf_fwd_statt1 = 0;
		tx_rf_fwd_statt2 = 0;
	}
	set_reg_176();
	return true; //this function always returns true. Maximum gain is 45
}
void rfic ::set_fb_gain(float gain){
	// Set Feedback path gain
	// @param gain){ output gain in dB
	//  parameters){
	// CartesianFBpathAmp1Gain, CartesianFBpathAmp2Gain,
	// CartesianFBpathAmp3Gain, CartesianFBpathAmp4Gain
	// (40 dB of range)
	// 5 dB steps
	// FIXME
	if (gain < 0.0)gain = 0.0;
	if (gain > 40.0) gain = 40.0;

	if (gain <= 2.5){
		CartesianFBpathAmp1Gain = 3;
		CartesianFBpathAmp2Gain = 3;
		CartesianFBpathAmp3Gain = 3;
		CartesianFBpathAmp4Gain = 3;
	}	
	else if ( gain > 2.5 && gain <= 7.5){
		CartesianFBpathAmp1Gain = 3;
		CartesianFBpathAmp2Gain = 3;
		CartesianFBpathAmp3Gain = 3;
		CartesianFBpathAmp4Gain = 1;
	}	
	else if ( gain > 7.5 && gain <= 12.5){
		CartesianFBpathAmp1Gain = 3;
		CartesianFBpathAmp2Gain = 3;
		CartesianFBpathAmp3Gain = 1;
		CartesianFBpathAmp4Gain = 1;
	}
	else if ( gain > 12.5 && gain <= 17.5){
		CartesianFBpathAmp1Gain = 3;
		CartesianFBpathAmp2Gain = 1;
		CartesianFBpathAmp3Gain = 1;
		CartesianFBpathAmp4Gain = 1;
	}
	else if ( gain > 17.5 && gain <= 22.5){
		CartesianFBpathAmp1Gain = 1;
		CartesianFBpathAmp2Gain = 1;
		CartesianFBpathAmp3Gain = 1;
		CartesianFBpathAmp4Gain = 1;
	}
	else if ( gain > 22.5 && gain <= 27.5){
		CartesianFBpathAmp1Gain = 1;
		CartesianFBpathAmp2Gain = 1;
		CartesianFBpathAmp3Gain = 1;
		CartesianFBpathAmp4Gain = 0;
	}
	else if ( gain > 27.5 && gain <= 32.5){
		CartesianFBpathAmp1Gain = 1;
		CartesianFBpathAmp2Gain = 1;
		CartesianFBpathAmp3Gain = 0;
		CartesianFBpathAmp4Gain = 0;
	}
	else if ( gain > 32.5 && gain <= 37.5){
		CartesianFBpathAmp1Gain = 1;
		CartesianFBpathAmp2Gain = 0;
		CartesianFBpathAmp3Gain = 0;
		CartesianFBpathAmp4Gain = 0;
	}
	else if ( gain > 37.5){
		CartesianFBpathAmp1Gain = 0;
		CartesianFBpathAmp2Gain = 0;
		CartesianFBpathAmp3Gain = 0;
		CartesianFBpathAmp4Gain = 0;
	}
	set_reg_169();
}

int* rfic :: calc_freq_vars(double _Fclk,double _Fout){
	//
	//@param Fclk: Clock frequency of board (Hz)
	//@type Fclk: float
	//@param Fout: Desired clock frequency for one of three frequency synthesizers (Hz)
	//@type Fout: float
	//
	// Calculate RFIC register variables to set frequency of frequency synthesizers
	// data1 corresponds to Ngt, D7, a single bit
	// data2 corresponds to NorNdiv4, D6-D0, up to seven bits
	// data3 corresponds to RorFrNpRdiv4, up to 26 bits
	// D7-D0, D7-D0, D7-D0, D7-D6
	// Returns Ngt, NorNdiv4, RorFrNpRdiv4_25to18, RorFrNpRdiv4_17to10,
	// RorFrNpRdiv4_9to2, RorFrNpRdiv4_1to0
	float NpR;
	int data1,data2,data3,temp,Ngt;
	int ret_arr[6];
	if (_Fout > _Fclk / 4){
		NpR = pow(2,-26) * floor(pow(2,26) * _Fclk / _Fout);
		data1 = 0;
		data2 = int(floor(NpR));
		data3 = int(pow(2,26) * (NpR - floor(NpR)));
	}
	else{
		NpR = pow(2,-24)* int(pow(2,24) * _Fclk / _Fout);
		data1 = 1;
		data2 = int(floor(NpR / 4));
		data3 = int(pow(2,26) * (NpR / 4 - floor(NpR / 4)));
	}

	Ngt = data1;//need to speak to terry about this variable..is this a local variable or supposed to be a class attribute
	ret_arr[0]= Ngt;
	NorNdiv4 = data2;
	ret_arr[1]=NorNdiv4;
	RorFrNpRdiv4_25to18 = data3 >> 18;
	ret_arr[2]=RorFrNpRdiv4_25to18;
	temp = data3 % int(pow(2,18));
	RorFrNpRdiv4_17to10 = temp >> 10;
	ret_arr[3]= RorFrNpRdiv4_17to10;
	temp = data3  % int(pow(2,10));
	RorFrNpRdiv4_9to2 = temp >> 2;
	ret_arr[4]= RorFrNpRdiv4_9to2;
	RorFrNpRdiv4_1to0 = data3 % int(pow(2,2));
	ret_arr[5]=RorFrNpRdiv4_1to0;

	return ret_arr;
}

int* rfic :: calc_phase_vars(double _Fclk, double _Fout,double phsh){
	//
	//@param _Fclk: Clock frequency of board (Hz)
	//@type _Fclk: float
	//@param _Fout: Desired clock frequency for one of three frequency synthesizers (Hz)
	//@type _Fout: float
	//@param phsh: Desired phase shift in degrees
	//@type phsh: float
	//
	// Calculate RFIC register variables to set phase of frequency synthesizers
	// data1 is NGT3_phsh, D7, a single bit
	// data2 is NorNdiv4_phsh, D6-D0, up to 7 bits
	// data3 is RorFrNpRdiv4_phsh, up to 26 bits
	// D7-D0, D7-D0, D7-D0, D7-D6
	// Returns Ngt_phsh, NorNdiv4_phsh, RorFrNpRdiv4_25to18_phsh, 
	// RorFrNpRdiv4_17to10_phsh, RorFrNpRdiv4_9to2_phsh, RorFrNpRdiv4_1to0_phsh

	float mod1,tmp,NpR,NpR_ph;
	int data1,data2,data3,temp,Ngt_phsh,RorFrNpRdiv4_17to10_phsh,RorFrNpRdiv4_9to2_phsh,RorFrNpRdiv4_25to18_phsh,RorFrNpRdiv4_1to0_phsh;
	int ret_arr[6];
	if (_Fout <= _Fclk / 4){
		mod1 = phsh - 360 * floor(phsh / 360);
		NpR = pow(2,-24) * int(pow(2,24) * _Fclk / _Fout);
		tmp = (1 + mod1 / 360 / 2) * NpR;
	}
	else{
		mod1 = phsh - 360 * floor(phsh / 360);
		NpR = pow(2,-26) * int(pow(2,26) * _Fclk / _Fout);
		tmp = (1 + mod1 / 360 / 2) * NpR;
	}
	if (tmp < 4){
		NpR_ph = pow(2,-26) * int(pow(2,26) * (1 + mod1 / 360 / 8) * NpR);
		data1 = 0;
		data2 = int(floor(NpR_ph));
		data3 = int(pow(2,26) * (NpR_ph - floor(NpR_ph)));
	}
	else if ((tmp >=4) && (tmp < 508)){
		NpR_ph = pow(2,-24) * int(pow(2,24) * tmp);
		data1 = 1;
		data2 = int(floor(NpR_ph / 4));
		data3 = int(pow(2,26) * (NpR_ph / 4 - floor(NpR_ph / 4)));
	}
	else if (tmp >= 508){
		NpR_ph = pow(2,-24) * int(pow(2,24) * (1 + (mod1 - 360) / 360 /2) * NpR);
		data1 = 1;
		data2 = int(floor(NpR_ph / 4));
		data3 = int(pow(2,26) * (NpR_ph / 4 - floor(NpR_ph / 4)));
	}
	Ngt_phsh = data1;
	ret_arr[0]=Ngt_phsh;
	NorNdiv4_phsh = data2;
	ret_arr[1]=NorNdiv4_phsh;
	RorFrNpRdiv4_25to18_phsh = data3 >> 18;
	ret_arr[2]=RorFrNpRdiv4_25to18_phsh;
	temp = data3 % int(pow(2,18));
	RorFrNpRdiv4_17to10_phsh = temp >> 10;
	ret_arr[3]=RorFrNpRdiv4_17to10_phsh;
	temp = data3 % int(pow(2,20));
	RorFrNpRdiv4_9to2_phsh = temp >> 2;
	ret_arr[4]=RorFrNpRdiv4_9to2_phsh;
	RorFrNpRdiv4_1to0_phsh = data3 % 4;
	ret_arr[5]=RorFrNpRdiv4_1to0_phsh;

	return ret_arr;
}


struct freq_result_t rfic :: set_rx_freq(double target_freq){
	//
	//@param target_freq: desired receiver frequency in Hz
	//@returns (ok, actual_baseb&&_freq) where:
	//   ok is True or False && indicates success or failure,
	//   actual_baseb&&_freq is the RF frequency that corresponds to DC in the IF.
	//

	// Go through Quadrature Generation Initialization Sequence
	struct freq_result_t args = {false, 0};
	//target_freq = target_freq + 4000000
	float try_freq;
	if (target_freq <= 500000000){
		// Below 500 MHz
		std::cout << "Below 500 MHz, divide by 2"<<std::endl;
		// Use QuIET frequency divided by 2
		// Step 1
		X1sel_32to39_3 = 0;
		X1sel_40to47_3 = 62;
		X2sel_32to36_3 = 0;
		X2sel_37to41_3 = 0;
		X4sel_32to36_3 = 0;
		X4sel_37to41_3 = 0;

		set_reg_139();
		set_reg_140();
		set_reg_141();
		set_reg_142();
		set_reg_143();
		set_reg_144();

		// Step 2
		X1sel_40to47_3 = 63;

		set_reg_140();

		try_freq = target_freq * 2;
		
	}
	else if ((target_freq > 500000000) && (target_freq <= 1000000000)){
		// Between 500 MHz && 1 GHz
		std::cout << "Between 500 MHz and 1 GHz"<<std::endl;
		// Use QuIET frequency
		// Step 1
		X1sel_32to39_3 = 1;
		X1sel_40to47_3 = 192;
		X2sel_32to36_3 = 0;
		X2sel_37to41_3 = 0;
		X4sel_32to36_3 = 0;
		X4sel_37to41_3 = 0;

		set_reg_139();
		set_reg_140();
		set_reg_141();
		set_reg_142();
		set_reg_143();
		set_reg_144();

		// Step 2
		X1sel_32to39_3 = 73;

		set_reg_139();

		// Step 3
		X1sel_32to39_3 = 201;

		set_reg_139();

		try_freq = target_freq;

		// Set Quadrature Generator Charge/Discharge Taps
		DischargeTap16_0to3_3 = 6;
		ChargeTap16_0to3_3 = 7;
		DischargeTapn_0to3_3 = 0;
		ChargeTapn16_0to3_3 = 5;

		// Set Quadrature Generator Delays
		Qg00degDelay_0to4_3 = 16;
		Qg90degDelay_0to4_3 = 31;
		Qg180degDelay_0to4_3 = 0;
		Qg270degDelay_0to4_3 = 31;

		set_reg_133();
		set_reg_134();
		set_reg_135();
		set_reg_136();
		set_reg_137();
		set_reg_138();
	}

	else if ((target_freq > 1000000000) && (target_freq <= 2000000000)){
		// Between 1 GHz && 2 GHz
		std::cout << "Between 1 GHz and 2 GHz, multiply by 2"<< std::endl;
		// Use QuIET multiplied by 2
		// Step 1
		X1sel_32to39_3 = 0;
		X1sel_40to47_3 = 0;
		X2sel_32to36_3 = 0;
		X2sel_37to41_3 = 7;
		X4sel_32to36_3 = 0;
		X4sel_37to41_3 = 0;

		set_reg_139();
		set_reg_140();
		set_reg_141();
		set_reg_142();
		set_reg_143();
		set_reg_144();

		// Step 2
		X2sel_32to36_3 = 9;

		set_reg_141();

		// Step 3
		X2sel_32to36_3 = 25;

		set_reg_141();

		// Step 4
		X2sel_32to36_3 = 16;

		set_reg_141();

		try_freq = target_freq / 2;

		// Set Quadrature Generator Charge/Discharge Taps
		DischargeTap16_0to3_3 = 9;
		ChargeTap16_0to3_3 = 3;
		DischargeTapn_0to3_3 = 3;
		ChargeTapn16_0to3_3 = 5;

		// Set Quadrature Generator Delays
		Qg00degDelay_0to4_3 = 31;
		Qg90degDelay_0to4_3 = 31;
		Qg180degDelay_0to4_3 = 0;
		Qg270degDelay_0to4_3 = 31;

		set_reg_133();
		set_reg_134();
		set_reg_135();
		set_reg_136();
		set_reg_137();
		set_reg_138();
	}

	else if ((target_freq > 2000000000.0) && (target_freq <= 4000000000.0)){
		// 2 to 4 GHz
		std::cout<< "From 2 to 4 GHz, multiply by 4"<<std::endl;
		// Use QuIET frequency multiplied by 4
		// Step 1
		X1sel_32to39_3 = 0;
		X1sel_40to47_3 = 0;
		X2sel_32to36_3 = 0;
		X2sel_37to41_3 = 0;
		X4sel_32to36_3 = 0;
		X4sel_37to41_3 = 7;

		set_reg_139();
		set_reg_140();
		set_reg_141();
		set_reg_142();
		set_reg_143();
		set_reg_144();

		// Step 2
		X4sel_32to36_3 = 9;

		set_reg_143();

		// Step 3
		X4sel_32to36_3 = 25;

		set_reg_143();

		try_freq = target_freq / 4;

		// Set Quadrature Generator Charge/Discharge Taps
		DischargeTap16_0to3_3 = 16;
		ChargeTap16_0to3_3 = 0;
		DischargeTapn_0to3_3 = 7;
		ChargeTapn16_0to3_3 = 7;

		// Set Quadrature Generator Delays
		Qg00degDelay_0to4_3 = 0;
		Qg90degDelay_0to4_3 = 31;
		Qg180degDelay_0to4_3 = 0;
		Qg270degDelay_0to4_3 = 31;

		set_reg_133();
		set_reg_134();
		set_reg_135();
		set_reg_136();
		set_reg_137();
		set_reg_138();
	}

	else if (target_freq > 4000000000.0){
		// Above 4 GHz, doesn't work
		args.ok = false;
		args.baseband_freq = target_freq;
		return args;
	}
		// FIXME
		/*// Above 4 GHz
		std::cout<< "Above 4 GHz, multiply by 8"<<std::endl;
		// Use QuIET frequency multiplied by 8
		// Step 1
		X1sel_32to39_3 = 0;
		X1sel_40to47_3 = 0;
		X2sel_32to36_3 = 0;
		X2sel_37to41_3 = 0;
		X4sel_32to36_3 = 0;
		X4sel_37to41_3 = 0;
		X8sel_32to36_3 = 0;
		X8sel_41_3 = 0;
		X8sel_37to40_3 = 7;

		set_reg_139();
		set_reg_140();
		set_reg_141();
		set_reg_142();
		set_reg_143();
		set_reg_144();
		set_reg_145();
		set_reg_146();

		// Step 2
		X8sel_32to36_3 = 9;

		set_reg_145();

		// Step 3
		X8sel_32to36_3 = 25;

		set_reg_145();

		try_freq = target_freq / 8;

		// Set Quadrature Generator Charge/Discharge Taps
		ChargeTap16_0to3_3 = 15;
		ChargeTapn16_0to3_3 = 15;

		DischargeTap16_0to3_3 = 6;
		DischargeTapn16_0to3_3 = 4;

		set_reg_137();
		set_reg_138();*/


	Foutrx = target_freq;
	int * ret_arr;
	ret_arr= calc_freq_vars(Fclk, try_freq);
	Ngt3_3 = ret_arr[0];
	NorNdiv4_3= ret_arr[1];
	RorFrNpRdiv4_25to18_3= ret_arr[2];
	RorFrNpRdiv4_17to10_3= ret_arr[3];
	RorFrNpRdiv4_9to2_3= ret_arr[4];
	RorFrNpRdiv4_1to0_3= ret_arr[5];

	set_reg_104();
	set_reg_105();
	set_reg_106();
	set_reg_107();
	set_reg_108();

	args.ok = true;
	args.baseband_freq = target_freq;
	return args;
	//FIXME -- How do I know if the RFIC successfully attained the desired frequency?//

}

struct freq_result_t rfic :: set_tx_freq(double target_freq){
	//
	//@param target_freq: desired transmitter frequency in Hz
	//@returns (ok, actual_baseb&&_freq) where:
	//   ok is True or False && indicates success or failure,
	//   actual_baseb&&_freq is the RF frequency that corresponds to DC in the IF.
	//

	// Go through Quadrature Generation Initialization Sequence

	// FIXME
	//target_freq = target_freq + 4000000
	//target_freq = target_freq + 1000000
	
	struct freq_result_t args = {false, 0};

	double try_freq;
	if (target_freq <= 500000000.0){
		std::cout<< "Below 500 MHz, divide by 2"<<std::endl;
		// Use QuIET frequency divided by 2
		// Step 1
		X1sel_32to39 = 0;
		X1sel_40to47 = 62;
		X2sel_32to36 = 0;
		X2sel_37to41 = 0;
		X4sel_32to36 = 0;
		X4sel_37to41 = 0;

		set_reg_35();
		set_reg_36();
		set_reg_37();
		set_reg_38();
		set_reg_39();
		set_reg_40();

		// Step 2
		X1sel_40to47 = 63;

		set_reg_36();

		try_freq = target_freq * 2;
	}
	else if ((target_freq > 500000000.0) && (target_freq <= 1000000000.0)){
		std::cout<< "Between 500 MHz and 1 GHz"<<std::endl;
		// Use QuIET frequency
		// Step 1
		X1sel_32to39 = 1;
		X1sel_40to47 = 192;
		X2sel_32to36 = 0;
		X2sel_37to41 = 0;
		X4sel_32to36 = 0;
		X4sel_37to41 = 0;

		set_reg_35();
		set_reg_36();
		set_reg_37();
		set_reg_38();
		set_reg_39();
		set_reg_40();

		// Step 2
		X1sel_32to39 = 73;

		set_reg_35();

		// Step 3
		X1sel_32to39 = 201;

		set_reg_35();

		try_freq = target_freq;

		// Set Quadrature Generator Charge/Discharge Taps && Delays
		Qg00degDelay_0to4 = 15;
		Qg90degDelay_0to4 = 12;
		Qg180degDelay_0to4 = 3;
		Qg270degDelay_0to4 = 12;

		set_reg_29();
		set_reg_30();
		set_reg_31();
		set_reg_32();

		DischargeTap16_0to3 = 1;
		ChargeTap16_0to3 = 8;
		DischargeTapn_0to3 = 7;
		ChargeTapn16_0to3 = 0;

		set_reg_33();
		set_reg_34();
	}
	else if ((target_freq > 1000000000.0) && (target_freq <= 2000000000.0)){
		std::cout<<"Between 1 GHz and 2 GHz, multiply by 2"<<std::endl;
		// Use QuIET multiplied by 2
		// Step 1
		X1sel_32to39 = 0;
		X1sel_40to47 = 0;
		X2sel_32to36 = 0;
		X2sel_37to41 = 7;
		X4sel_32to36 = 0;
		X4sel_37to41 = 0;

		set_reg_35();
		set_reg_36();
		set_reg_37();
		set_reg_38();
		set_reg_39();
		set_reg_40();

		// Step 2
		X2sel_32to36 = 9;

		set_reg_37();

		// Step 3
		X2sel_32to36 = 25;

		set_reg_37();

		// Step 4
		//X2sel_32to36 = 16;

		//set_reg_37();

		try_freq = target_freq / 2;

		// Set Quadrature Generator Charge/Discharge Taps && Delays
		Qg00degDelay_0to4 = 7;
		Qg90degDelay_0to4 = 8;
		Qg180degDelay_0to4 = 7;
		Qg270degDelay_0to4 = 5;

		set_reg_29();
		set_reg_30();
		set_reg_31();
		set_reg_32();

		DischargeTap16_0to3 = 1;
		ChargeTap16_0to3 = 13;

		DischargeTapn_0to3 = 3;
		ChargeTapn16_0to3 = 9;

		set_reg_33();
		set_reg_34();
	}
	else if ((target_freq > 2000000000.0) && (target_freq <= 4000000000.0)){
		std::cout<<"2-4 GHz, multiply by 4"<<std::endl;
		// Use QuIET frequency multiplied by 4
		// Step 1
		X1sel_32to39 = 0;
		X1sel_40to47 = 0;
		X2sel_32to36 = 0;
		X2sel_37to41 = 0;
		X4sel_32to36 = 0;
		X4sel_37to41 = 7;

		set_reg_35();
		set_reg_36();
		set_reg_37();
		set_reg_38();
		set_reg_39();
		set_reg_40();

		// Step 2
		X4sel_32to36 = 9;

		set_reg_39();

		// Step 3
		X4sel_32to36 = 25;

		set_reg_39();

		try_freq = target_freq / 4;

		// Set Quadrature Generator Charge/Discharge Taps && Delays
		Qg00degDelay_0to4 = 0;
		Qg90degDelay_0to4 = 17;
		Qg180degDelay_0to4 = 15;
		Qg270degDelay_0to4 = 20;

		set_reg_29();
		set_reg_30();
		set_reg_31();
		set_reg_32();

		DischargeTap16_0to3 = 15;
		ChargeTap16_0to3 = 0;

		DischargeTapn_0to3 = 10;
		ChargeTapn16_0to3 = 8;

		set_reg_33();
		set_reg_34();
	}
	else if (target_freq > 4000000000.0){
		// Above 4 GHz, doesn't work
		args.ok = false;
		args.baseband_freq = target_freq;
		return args;
	}

	Fouttx = target_freq;

	int * ret_arr;
	ret_arr= calc_freq_vars(Fclk, try_freq);
	Ngt3 = ret_arr[0];
	NorNdiv4= ret_arr[1];
	RorFrNpRdiv4_25to18= ret_arr[2];
	RorFrNpRdiv4_17to10= ret_arr[3];
	RorFrNpRdiv4_9to2= ret_arr[4];
	RorFrNpRdiv4_1to0= ret_arr[5];

	set_reg_0();
	set_reg_1();
	set_reg_2();
	set_reg_3();
	set_reg_4();
	
	args.ok = true;
	args.baseband_freq = target_freq;
	return args;
	//FIXME -- How do I know if the RFIC successfully attained the desired frequency?//

}

bool rfic :: set_fb_freq(double target_freq){
	//
	//@param target_freq: desired transmitter frequency in Hz
	//@returns (ok, actual_baseb&&_freq) where:
	//   ok is True or False && indicates success or failure,
	//   actual_baseb&&_freq is the RF frequency that corresponds to DC in the IF.
	//

	// Go through Quadrature Generation Initialization Sequence
	double try_freq;
	if (target_freq <= 500000000.0){
		std::cout<<"Below 500 MHz, divide by 2"<<std::endl;
		// Use QuIET frequency divided by 2
		// Step 1
		X1sel_32to39_2 = 0;
		X1sel_40to47_2 = 62;
		X2sel_32to36_2 = 0;
		X2sel_37to41_2 = 0;
		X4sel_32to36_2 = 0;
		X4sel_37to41_2 = 0;

		set_reg_83();
		set_reg_84();
		set_reg_85();
		set_reg_86();
		set_reg_87();
		set_reg_88();

		// Step 2
		X1sel_40to47_2 = 63;

		set_reg_84();

		try_freq = target_freq * 2;
	}
	else if ((target_freq > 500000000) && (target_freq <= 1000000000)){
		std::cout<< "Between 500 MHz and 1 GHz"<<std::endl;
		// Use QuIET frequency
		// Step 1
		X1sel_32to39_2 = 1;
		X1sel_40to47_2 = 192;
		X2sel_32to36_2 = 0;
		X2sel_37to41_2 = 0;
		X4sel_32to36_2 = 0;
		X4sel_37to41_2 = 0;

		set_reg_83();
		set_reg_84();
		set_reg_85();
		set_reg_86();
		set_reg_87();
		set_reg_88();

		// Step 2
		X1sel_32to39_2 = 73;

		set_reg_83();

		// Step 3
		X1sel_32to39_2 = 201;

		set_reg_83();

		try_freq = target_freq;

		// Set Quadrature Generator Charge/Discharge Taps
		// FIXME
		//ChargeTap16_0to3_2 = 7;
		//ChargeTapn16_0to3_2 = 5;

		//DischargeTap16_0to3_2 = 6;
		//DischargeTapn16_0to3_2 = 0;

		//set_reg_81();
		//set_reg_82();
	}

	else if ((target_freq > 1000000000) && (target_freq <= 2000000000)){
		std::cout<<"Between 1 GHz and 2 GHz, multiply by 2"<<std::endl;
		// Use QuIET multiplied by 2
		// Step 1
		X1sel_32to39_2 = 0;
		X1sel_40to47_2 = 0;
		X2sel_32to36_2 = 0;
		X2sel_37to41_2 = 7;
		X4sel_32to36_2 = 0;
		X4sel_37to41_2 = 0;

		set_reg_83();
		set_reg_84();
		set_reg_85();
		set_reg_86();
		set_reg_87();
		set_reg_88();

		// Step 2
		X2sel_32to36_2 = 9;

		set_reg_85();

		// Step 3
		X2sel_32to36_2 = 25;

		// Step 4
		//X2sel_32to36 = 16;

		set_reg_85();

		try_freq = target_freq / 2;

		// Set Quadrature Generator Charge/Discharge Taps
		// FIXME
		//ChargeTap16_0to3_2 = 7;
		//ChargeTapn16_0to3_2 = 8;

		//DischargeTap16_0to3_2 = 15;
		//DischargeTapn16_0to3_2 = 15;

		//set_reg_81();
		//set_reg_82();
	}
	else if ((target_freq > 2000000000.0) && (target_freq <= 4000000000.0)){
		std::cout<<"2-4 GHz, multiply by 4"<<std::endl;
		// Use QuIET frequency multiplied by 4
		// Step 1
		X1sel_32to39_2 = 0;
		X1sel_40to47_2 = 0;
		X2sel_32to36_2 = 0;
		X2sel_37to41_2 = 0;
		X4sel_32to36_2 = 0;
		X4sel_37to41_2 = 7;

		set_reg_83();
		set_reg_84();
		set_reg_85();
		set_reg_86();
		set_reg_87();
		set_reg_88();

		// Step 2
		X4sel_32to36_2 = 9;

		set_reg_87();

		// Step 3
		X4sel_32to36_2 = 25;

		set_reg_87();

		try_freq = target_freq / 4;

		// Set Quadrature Generator Charge/Discharge Taps
		// FIXME
		//ChargeTap16_0to3_2 = 15;
		//ChargeTapn16_0to3_2 = 15;

		//DischargeTap16_0to3_2 = 6;
		//DischargeTapn16_0to3_2 = 4;

		//set_reg_81();
		//set_reg_82();
	}
	else if (target_freq > 4000000000.0){
		// Above 4 GHz, doesn't work
		return false;
	}
	Foutfb = target_freq;
	int * ret_arr;
	ret_arr= calc_freq_vars(Fclk, try_freq);
	Ngt3_2 = ret_arr[0];
	NorNdiv4_2= ret_arr[1];
	RorFrNpRdiv4_25to18_2= ret_arr[2];
	RorFrNpRdiv4_17to10_2= ret_arr[3];
	RorFrNpRdiv4_9to2_2= ret_arr[4];
	RorFrNpRdiv4_1to0_2= ret_arr[5];


	set_reg_48();
	set_reg_49();
	set_reg_50();
	set_reg_51();
	set_reg_52();

	return true;
	//FIXME -- How do I know if the RFIC successfully attained the desired frequency?//
}

/////////////////////////////////////////

bool rfic :: set_rx_phase(int phsh){
	//
	//@param phsh{ desired phase shift in degrees
	//@returns (ok) where{
	//   ok is True or False and indicates success or failure
	//
	double synth_freq;
	phsh = phsh % 360;

	if (Foutrx <= 500000000.0)
		synth_freq = Foutrx * 2;
	else if ( (Foutrx > 500000000.0) and (Foutrx <= 1000000000.0))
		synth_freq = Foutrx;
	else if ( (Foutrx > 1000000000.0) and (Foutrx < 2000000000.0))
		synth_freq = Foutrx / 2;
	else if (Foutrx > 2000000000.0)
		synth_freq = Foutrx / 4;

	int * ret_arr;
	ret_arr= calc_phase_vars(Fclk, synth_freq,phsh);
	Qu_tx_Ngt3_3 = ret_arr[0];
	NorNdiv4_phsh_3= ret_arr[1];
	RorFrNpRdiv4_phsh_25to18_3= ret_arr[2];
	RorFrNpRdiv4_phsh_17to10_3= ret_arr[3];
	RorFrNpRdiv4_phsh_9to2_3= ret_arr[4];
	RorFrNpRdiv4_phsh_1to0_3= ret_arr[5];


	set_reg_109();
	set_reg_110();
	set_reg_111();
	set_reg_112();
	set_reg_113();

	return true;
	//FIXME -- How do I know if the RFIC successfully attained the desired phase?//
}

bool rfic :: set_tx_phase(int phsh){
	//
	//@param phsh{ desired phase shift in degrees
	//@returns (ok) where{
	//   ok is True or False and indicates success or failure
	//
	double synth_freq;
	phsh = phsh % 360;

	if (Fouttx <= 500000000.0)
		synth_freq = Fouttx * 2;
	else if ( (Fouttx > 500000000.0) and (Fouttx <= 1000000000.0))
		synth_freq = Fouttx;
	else if ( (Fouttx > 1000000000.0) and (Fouttx < 2000000000.0))
		synth_freq = Fouttx / 2;
	else if (Fouttx > 2000000000)
		synth_freq = Fouttx / 4;

	int * ret_arr;
	ret_arr= calc_phase_vars(Fclk, synth_freq,phsh);
	Qu_tx_Ngt3_3 = ret_arr[0];
	NorNdiv4_phsh_3= ret_arr[1];
	RorFrNpRdiv4_phsh_25to18_3= ret_arr[2];
	RorFrNpRdiv4_phsh_17to10_3= ret_arr[3];
	RorFrNpRdiv4_phsh_9to2_3= ret_arr[4];
	RorFrNpRdiv4_phsh_1to0_3= ret_arr[5];

	set_reg_5();
	set_reg_6();
	set_reg_7();
	set_reg_8();
	set_reg_9();

	//FIXME -- How do I know if the RFIC successfully attained the desired phase?//
	return true;
}

bool rfic :: set_fb_phase(int phsh){
	//
	//@param phsh{ desired phase shift in degrees
	//@returns (ok) where{
	//   ok is True or False and indicates success or failure
	//
	double synth_freq;
	phsh = phsh % 360;

	if (Foutfb <= 500000000.0)
		synth_freq = Foutfb * 2;
	else if ( (Foutfb > 500000000.0) and (Foutfb <= 1000000000.0))
		synth_freq = Foutfb;
	else if ( (Foutfb > 1000000000.0) and (Foutfb < 2000000000.0))
		synth_freq = Foutfb / 2;
	else if (Foutfb > 2000000000.0)
		synth_freq = Foutfb / 4;

	
	int * ret_arr;
	ret_arr= calc_phase_vars(Fclk, synth_freq,phsh);
	Qu_tx_Ngt3_3 = ret_arr[0];
	NorNdiv4_phsh_3= ret_arr[1];
	RorFrNpRdiv4_phsh_25to18_3= ret_arr[2];
	RorFrNpRdiv4_phsh_17to10_3= ret_arr[3];
	RorFrNpRdiv4_phsh_9to2_3= ret_arr[4];
	RorFrNpRdiv4_phsh_1to0_3= ret_arr[5];

	set_reg_53();
	set_reg_54();
	set_reg_55();
	set_reg_56();
	set_reg_57();

	//FIXME -- How do I know if the RFIC successfully attained the desired phase?//
	return true;
}

bool rfic ::  set_rx_bw(float bw){
	//
	//@param bw{ desired bandwidth in Hz
	//
	// Available bandwidth{ 4.25 kHz to 14 MHz (baseband)
	// FIXME
	std::cout<<"Desired bandwidth :"<< bw<<std::endl;
	if (bw <= 5250){
		// Set BW to 3.532 kHz
		rx_rfp = 3;
		rx_cp_12to8 = 31;
		rx_cp_7to0 = 240;

		rx_rv = 7;
		rx_cv_10to3 = 254;
		rx_cv_2to0 = 0;

		rx_rq = 7;
		rx_cq_9to8 = 3;
		rx_cq_7to0 = 240;
	}

	else if (bw > 5250 and bw <= 10500){
		// Set BW to 7.065 kHz
		rx_rfp = 3;
		rx_cp_12to8 = 31;
		rx_cp_7to0 = 240;

		rx_rv = 5;
		rx_cv_10to3 = 254;
		rx_cv_2to0 = 0;

		rx_rq = 5;
		rx_cq_9to8 = 3;
		rx_cq_7to0 = 240;
	}
	else if (bw > 10500 and bw <= 21000){
		// Set BW to 14.130 kHz
		rx_rfp = 2;
		rx_cp_12to8 = 31;
		rx_cp_7to0 = 240;

		rx_rv = 4;
		rx_cv_10to3 = 254;
		rx_cv_2to0 = 0;

		rx_rq = 4;
		rx_cq_9to8 = 3;
		rx_cq_7to0 = 240;
	}
	
	else if (bw > 21000 and bw <= 42000){
		// Set BW to 28.259 kHz
		rx_rfp = 2;
		rx_cp_12to8 = 15;
		rx_cp_7to0 = 240;

		rx_rv = 3;
		rx_cv_10to3 = 254;
		rx_cv_2to0 = 0;

		rx_rq = 3;
		rx_cq_9to8 = 3;
		rx_cq_7to0 = 240;
	}
	else if (bw > 42000 and bw <= 84500){
		// Set BW to 56.518 kHz
		rx_rfp = 2;
		rx_cp_12to8 = 7;
		rx_cp_7to0 = 240;

		rx_rv = 2;
		rx_cv_10to3 = 254;
		rx_cv_2to0 = 0;

		rx_rq = 2;
		rx_cq_9to8 = 3;
		rx_cq_7to0 = 240;
	}
	else if (bw > 84500 and bw <= 169500){
		// Set BW to 113.036 kHz
		rx_rfp = 2;
		rx_cp_12to8 = 3;
		rx_cp_7to0 = 240;

		rx_rv = 1;
		rx_cv_10to3 = 254;
		rx_cv_2to0 = 0;

		rx_rq = 1;
		rx_cq_9to8 = 3;
		rx_cq_7to0 = 240;
	}
	else if (bw > 169500 and bw <= 339000){
		// Set BW to 226.072 kHz
		rx_rfp = 2;
		rx_cp_12to8 = 1;
		rx_cp_7to0 = 240;

		rx_rv = 1;
		rx_cv_10to3 = 126;
		rx_cv_2to0 = 0;

		rx_rq = 1;
		rx_cq_9to8 = 1;
		rx_cq_7to0 = 240;
	}
	else if (bw > 339000 and bw <= 667000){
		// Set BW to 452.145 kHz
		rx_rfp = 1;
		rx_cp_12to8 = 1;
		rx_cp_7to0 = 240;

		rx_rv = 0;
		rx_cv_10to3 = 254;
		rx_cv_2to0 = 0;

		rx_rq = 1;
		rx_cq_9to8 = 0;
		rx_cq_7to0 = 240;
	}
	else if (bw > 667000 and bw <= 1356000){
		// Set BW to 904.289 kHz
		rx_rfp = 1;
		rx_cp_12to8 = 0;
		rx_cp_7to0 = 240;

		rx_rv = 0;
		rx_cv_10to3 = 126;
		rx_cv_2to0 = 0;

		rx_rq = 0;
		rx_cq_9to8 = 3;
		rx_cq_7to0 = 240;
	}
	else if (bw > 1356000 and bw <= 2712500){
		// Set BW to 1808.579 kHz
		rx_rfp = 1;
		rx_cp_12to8 = 0;
		rx_cp_7to0 = 112;

		rx_rv = 0;
		rx_cv_10to3 = 62;
		rx_cv_2to0 = 0;

		rx_rq = 0;
		rx_cq_9to8 = 1;
		rx_cq_7to0 = 240;
	}
	else if (bw > 2712500 and bw <= 5425500){
		// Set BW to 3617.157 kHz
		rx_rfp = 0;
		rx_cp_12to8 = 0;
		rx_cp_7to0 = 112;

		rx_rv = 0;
		rx_cv_10to3 = 30;
		rx_cv_2to0 = 0;

		rx_rq = 0;
		rx_cq_9to8 = 0;
		rx_cq_7to0 = 240;
	}
	else if (bw > 5425500 and bw <= 10851000){
		// Set BW to 7234.315 kHz
		rx_rfp = 0;
		rx_cp_12to8 = 0;
		rx_cp_7to0 = 48;

		rx_rv = 0;
		rx_cv_10to3 = 14;
		rx_cv_2to0 = 0;

		rx_rq = 0;
		rx_cq_9to8 = 0;
		rx_cq_7to0 = 112;
	}
	else if (bw > 10851000){
		// Set BW to 14468.630 kHz
		rx_rfp = 0;
		rx_cp_12to8 = 0;
		rx_cp_7to0 = 16;

		rx_rv = 0;
		rx_cv_10to3 = 6;
		rx_cv_2to0 = 0;

		rx_rq = 0;
		rx_cq_9to8 = 0;
		rx_cq_7to0 = 48;
	}
	set_reg_198();
	set_reg_199();
	set_reg_200();
	set_reg_201();
	set_reg_202();
	set_reg_203();
	set_reg_204();
	return true; //always returning true..may have to change this

}
	


bool rfic :: set_tx_bw(float bw){
	//
	//@param bw{ desired bandwidth in Hz
	//
	// Available bandwidth{ 6.25 kHz to 14+ MHz (baseband)
	// FIXME
	std::cout<<"Desired bandwidth :"<< bw<<std::endl;
	if (bw <= 20000){
	// Set BW to 12.5 kHz
	tx_p1_bw = 3;
		tx_p2_bw2 = 15;
	}
	else if (bw > 20000 and bw <= 37500){
		// Set BW to 25 kHz
		tx_p1_bw = 3;
		tx_p2_bw2 = 7;
	}	
	else if (bw > 37500 and bw <= 75000){
		// Set BW to 50 kHz
		tx_p1_bw = 3;
		tx_p2_bw2 = 3;
	}
	else if (bw > 75000 and bw <= 150000){
		// Set BW to 100 kHz
		tx_p1_bw = 3;
		tx_p2_bw2 = 1;
	}
	else if (bw > 150000 and bw <= 425000){
		// Set BW to 200 kHz
		tx_p1_bw = 3;
		tx_p2_bw2 = 0;
	}
	else if (bw > 425000 and bw <= 1125000){
		// Set BW to 750 kHz
		tx_p1_bw = 1;
		tx_p2_bw2 = 15;
	}
	else if (bw > 1125000 and bw <= 2250000){
		// Set BW to 1.5 MHz
		tx_p1_bw = 1;
		tx_p2_bw2 = 7;
	}
	else if (bw > 2250000 and bw <= 4500000){
		// Set BW to 3 MHz
		tx_p1_bw = 1;
		tx_p2_bw2 = 3;
	}
	else if (bw > 4500000 and bw <= 9000000){
		// Set BW to 6 MHz
	tx_p1_bw = 1;
		tx_p2_bw2 = 1;
	}
	else if (bw > 9000000 and bw <= 13000000){
		// Set BW to 12 MHz
		tx_p1_bw = 1;
		tx_p2_bw2 = 0;
	}
	else if (bw > 13000000){
		// Set BW to 14+ MHz
		tx_p1_bw = 0;
		tx_p2_bw2 = 0;
	}
	set_reg_173();
	set_reg_174();
	return true; //always returning true..may have to change this
}

void rfic :: set_fb_bw(float bw){
	//
	//@param bw{ desired bandwidth in Hz
	//
	// Available bandwidth{ 5 MHz to 14+ MHz (baseband)
	// FIXME
	std::cout<<"Desired bandwidth :"<< bw<<std::endl;
	if (bw <= 7500000){
		// Set BW to 5 MHz
		tx_bb_fdbk_bw = 3;
	}
	else if (bw > 7500000 and bw <= 12000000){
		// Set BW to 10 MHz
		tx_bb_fdbk_bw = 1;
	}
	else if (bw > 12000000){
		// Set BW to 14+ MHz
		tx_bb_fdbk_bw = 0;
	}
	set_reg_156();
}

void rfic :: enable_tx_fb(){
	//
	// Enable transmitter feedback to RX port for DC offset correction, etc.
	//
	// FIXME
	std::cout<<"Enabling Transmit Feedback"<<std::endl;

	// Disable RX Filter
	rx_foe = 0;
	set_reg_196();

	// Enable Baseband Feedback, TX I and Q via RX I and Q
	tx_bb_fdbk_en = 3;
	set_reg_157();

	// Disable Baseband Feedback Calibration
	// FIXME
	//tx_bb_fdbk_cal_en = 0

	// Enable Baseband Feedback Cartesian Forward Path
	tx_bb_fdbk_cart_fwd_en = 1;
	set_reg_156();

	// Enable Cartesian Feedback Path
	tx_cart_en = 1;
	set_reg_160();

	// Enable Cartesian Feedback
	CartesianFeedbackpathenable = 1;

	// Enable Cartesian Feedback Path DCOC
	CartesianFeedbackpathDCOCenable = 1;
	set_reg_166();

	// Set Cartesian Feedback Path Amplifier Gain
	CartesianFBpathAmp1Gain = 0;
	CartesianFBpathAmp2Gain = 0;
	CartesianFBpathAmp3Gain = 0;
	CartesianFBpathAmp4Gain = 0;
	set_reg_169();

	// Enable Cartesian Feedback Path Zero
	CartesianFBpathZeroEnable = 1;
	set_reg_170();
}

void rfic :: disable_tx_fb(){
	//
	// Disable transmitter feedback to RX port
	//
	// FIXME
	std::cout<<"Disabling Transmit Feedback"<<std::endl;

	// Enable RX Filter
	rx_foe = 1;
	set_reg_196();

	// Disable Baseband Feedback
	tx_bb_fdbk_en = 0;
	set_reg_157();

	// Enable Baseband Feedback Calibration
	// FIXME
	//tx_bb_fdbk_cal_en = 1

	// Disable Baseband Feedback Cartesian Forward Path
	tx_bb_fdbk_cart_fwd_en = 0;
	set_reg_156();

	// Disable Cartesian Feedback Path
	tx_cart_en = 0;
	set_reg_160();

	// Disable Cartesian Feedback
	CartesianFeedbackpathenable = 0;

	// Disable Cartesian Feedback Path DCOC
	CartesianFeedbackpathDCOCenable = 0;
	set_reg_166();

	// Set Cartesian Feedback Path Amplifier Gain
	CartesianFBpathAmp1Gain = 3;
	CartesianFBpathAmp2Gain = 3;
	CartesianFBpathAmp3Gain = 3;
	CartesianFBpathAmp4Gain = 3;
	set_reg_169();

	// Disable Cartesian Feedback Path Zero
	CartesianFBpathZeroEnable = 0;
	set_reg_170();
}


int rfic :: RSSI_fade(){
	// Return fade, clip from the two RX-side ADCs.
	//@returns fade, clip
	// variables proportional to how much fading (low signal strength)
	// or clipping (high signal strength) is going on

	// Turn off test mux
	int fade;
	TestMuxBufferEnable = 0; //Disable Test Mux Buffer
	TestMuxEnable = 0; //Disable Test Mux
	TestMuxSetting = 0; //Four Output Description (Test1, Test2, Test3, Test4)
	set_reg_222();

	// Turn on on-channel detectors
	// Off-channel doesn't work - leave it off
	rx_onchen = 1; //Enables on-channel detector.
	rx_offchen = 0; //Disables off-channel detector
	set_reg_196();

	// Set clip and fade thresholds
	rx_offch = 1; //Sets the Clip Threshold for the Off-channel Detector
	rx_onchf = 0; //Sets the Fade Threshold for the On-channel Detector relative to the On-channel clip point.
	rx_onchc = 2; //Sets the Clip Threshold for the On-channel Detector
	set_reg_197();

	fade = usrp()->read_aux_adc(d_which, 0);
	return fade; 
}
int rfic :: RSSI_clip(){
	// Return fade, clip from the two RX-side ADCs.
	//@returns fade, clip
	// variables proportional to how much fading (low signal strength)
	// or clipping (high signal strength) is going on

	// Turn off test mux
	int clip;
	TestMuxBufferEnable = 0; //Disable Test Mux Buffer
	TestMuxEnable = 0; //Disable Test Mux
	TestMuxSetting = 0; //Four Output Description (Test1, Test2, Test3, Test4)
	set_reg_222();

	// Turn on on-channel detectors
	// Off-channel doesn't work - leave it off
	rx_onchen = 1; //Enables on-channel detector.
	rx_offchen = 0; //Disables off-channel detector
	set_reg_196();

	// Set clip and fade thresholds
	rx_offch = 1; //Sets the Clip Threshold for the Off-channel Detector
	rx_onchf = 0; //Sets the Fade Threshold for the On-channel Detector relative to the On-channel clip point.
	rx_onchc = 2; //Sets the Clip Threshold for the On-channel Detector
	set_reg_197();

	clip = usrp()->read_aux_adc(d_which, 1);
	return clip; 
}


/*****************************************************************************/


struct rfic_table_entry {
  rfic_key 			key;
  boost::weak_ptr<rfic>		value;

  rfic_table_entry(const rfic_key &_key, boost::weak_ptr<rfic> _value)
    : key(_key), value(_value) {}
};

typedef std::vector<rfic_table_entry> rfic_table;

static boost::mutex s_table_mutex;
static rfic_table s_table;

static rfic_sptr
_get_or_make_rfic(usrp_basic_sptr usrp, int which)
{
  rfic_key key = {usrp->serial_number(), which};

  boost::mutex::scoped_lock	guard(s_table_mutex);

  for (rfic_table::iterator p = s_table.begin(); p != s_table.end();){
    if (p->value.expired())	// weak pointer is now dead
      p = s_table.erase(p);	// erase it
    else {
      if (key == p->key){	// found it
	return rfic_sptr(p->value);
      }
      else		        
	++p;			// keep looking
    }
  }

  // We don't have the rfic we're looking for

  // create a new one and stick it in the table.
  rfic_sptr r(new rfic(usrp, which));
  rfic_table_entry t(key, r);
  s_table.push_back(t);

  return r;
}


/*****************************************************************************/



db_rfic_base::db_rfic_base(usrp_basic_sptr usrp, int which)
  : db_base(usrp, which)
{
  /*
   * Abstract base class for all rfic boards.
   * 
   * Derive board specific subclasses from db_rfic_base_{tx,rx}
   *
   * @param usrp: instance of usrp.source_c
   * @param which: which side: 0 or 1 corresponding to side A or B respectively
   * @type which: int
   */
  
  d_rfic = _get_or_make_rfic(usrp, which);
}

db_rfic_base::~db_rfic_base()
{
}


bool
db_rfic_base::is_quadrature()
{
  /*
   * Return True if this board requires both I & Q analog channels.
   *
   * This bit of info is useful when setting up the USRP Rx mux register.
   */
   return true;
}


double
db_rfic_base::freq_min()
{
  return 2.5e6;
}


double
db_rfic_base::freq_max()
{
  return 1e8;
}


void
db_rfic_base::shutdown_common()
{
  // If the usrp_basic in the rfic is the same as the usrp_basic
  // in the daughterboard, shutdown the rfic now (when only one of Tx
  // and Rx is open, this is always true).

  if (d_rfic->usrp() == usrp()){
    //std::cerr << "db_rfic_base::shutdown_common: same -> shutting down\n";
    d_rfic->shutdown();
  }
  else {
    //std::cerr << "db_rfic_base::shutdown_common: different -> ignoring\n";
  }
}

/******************************************************************************************************/

////////////
db_rfic_tx::db_rfic_tx(usrp_basic_sptr usrp, int which)
  : db_rfic_base(usrp, which)
{
	
	//select_tx_antenna(abc);
	d_rfic->set_reg_0();
	d_rfic->set_reg_1();
	d_rfic->set_reg_2();
	d_rfic->set_reg_3();
	d_rfic->set_reg_4();
	d_rfic->set_reg_5();
	d_rfic->set_reg_6();
	d_rfic->set_reg_7();
	d_rfic->set_reg_8();
	d_rfic->set_reg_9();
	d_rfic->set_reg_10();
	d_rfic->set_reg_12();
	d_rfic->set_reg_13();
	d_rfic->set_reg_14();
	d_rfic->set_reg_15();
	d_rfic->set_reg_16();
	d_rfic->set_reg_17();
	d_rfic->set_reg_18();
	d_rfic->set_reg_19();
	d_rfic->set_reg_20();
	d_rfic->set_reg_21();
	d_rfic->set_reg_22();
	d_rfic->set_reg_23();
	d_rfic->set_reg_24();
	d_rfic->set_reg_29();
	d_rfic->set_reg_30();
	d_rfic->set_reg_31();
	d_rfic->set_reg_32();
	d_rfic->set_reg_33();
	d_rfic->set_reg_34();
	d_rfic->set_reg_35();
	d_rfic->set_reg_36();
	d_rfic->set_reg_37();
	d_rfic->set_reg_38();
	d_rfic->set_reg_39();
	d_rfic->set_reg_40();
	d_rfic->set_reg_41();
	d_rfic->set_reg_42();
	d_rfic->set_reg_43();
	d_rfic->set_reg_156();
	d_rfic->set_reg_157();
	d_rfic->set_reg_158();
	d_rfic->set_reg_159();
	d_rfic->set_reg_160();
	d_rfic->set_reg_161();
	d_rfic->set_reg_162();
	d_rfic->set_reg_163();
	d_rfic->set_reg_164();
	d_rfic->set_reg_165();
	d_rfic->set_reg_166();
	d_rfic->set_reg_167();
	d_rfic->set_reg_168();
	d_rfic->set_reg_169();
	d_rfic->set_reg_170();
	d_rfic->set_reg_171();
	d_rfic->set_reg_172();
	d_rfic->set_reg_173();
	d_rfic->set_reg_174();
	d_rfic->set_reg_175();
	d_rfic->set_reg_176();
	d_rfic->set_reg_177();
	d_rfic->set_reg_178();
	d_rfic->set_reg_179();
	d_rfic->set_reg_180();
	d_rfic->set_reg_181();

	// Get digital block out of digital reset state
	d_rfic->Rst_n_async = 1;
	d_rfic->set_reg_24();

	// Turn on forward baseband reference section
	d_rfic->tx_bb_en = 1;
	// FIXME
	//d_rfic->set_reg_156();

	// Unroutes the Cartesian error signal through the BB Correction feedback
	// FIXME
	d_rfic->tx_bb_fdbk_cart_err_en = 0;

	// Routes the Cartesian feedback signal through the BB Correction feedback
	// FIXME
	d_rfic->tx_bb_fdbk_cart_fb_en = 1;
	d_rfic->set_reg_156();

	// Turn on baseband feedback section
	// FIXME
	//d_rfic->tx_bb_fdbk_en = 3;
	//d_rfic->set_reg_157();

	// Turn on forward RF transmit path
	d_rfic->RFForwardPathEnable_toMUX = 1;
	d_rfic->set_reg_175();

	// Turn on Cartesian FB path switch to forward summer
	d_rfic->CartesianFBpathSwitchtoforwardSummer = 1;
	d_rfic->set_reg_168();

	// Turn on Cartesian zero
	d_rfic->CartesianFBpathZeroEnable = 1;
	d_rfic->set_reg_170();

	// Select TX output path, default tx1
	// FIXME
	d_rfic->tx_output_channel_sel = 2;
	//d_rfic->tx_output_channel_sel = 1;
	d_rfic->set_reg_172();

	// Set TX Channel 1 Gain
	// The gain control on TX channel 1 is controlled by this DAC
	// The maximum voltage is 2.2 volts, which corresponds to 2750
	// This controls about 35 dB of gain ONLY ON TX 1
	d_rfic->usrp()->write_aux_dac(d_rfic->d_which, 3, 2750);


	// POR On.  This enables the clock that drives the digital block (which provides the tap selection process).  It must be enabled to generate an output.  See Byp_fine, address 10, bit 6
	d_rfic->Clk_driver_en = 1;

	// POR On
	d_rfic->qu_reg_en = 1;

	// POR On
	d_rfic->qq_reg_en = 1;

	// POR Off
	d_rfic->win_rst = 0;

	// POR On
	d_rfic->fineEn = 0;

	// POR Off
	d_rfic->fineEnb = 1;

	// POR On
	//d_rfic->rsffEn = 0;

	// POR On
	d_rfic->dl_en = 1;

	// POR On
	d_rfic->cp_en = 1;

	d_rfic->set_reg_20();
	d_rfic->set_reg_21();
}

db_rfic_tx::~db_rfic_tx()
{
  	// print "rfic_base_tx.__del__"
	// Power down

	// Turn off output channel
	d_rfic->tx_output_channel_sel = 0;
	d_rfic->set_reg_172();

	// Turn off forward RF transmit path
	d_rfic->RFForwardPathEnable_toMUX = 0;
	d_rfic->set_reg_17();

	// Turn off forward baseband reference section
	d_rfic->tx_bb_en = 0;
	d_rfic->set_reg_156();

	// Unroutes the Cartesian error signal through the BB Correction feedback
	// FIXME
	d_rfic->tx_bb_fdbk_cart_err_en = 0;

	// Unroutes the Cartesian feedback signal through the BB Correction feedback
	d_rfic->tx_bb_fdbk_cart_fb_en = 0;
	d_rfic->set_reg_156();

	// Turn off Cartesian FB path switch to forward summer
	d_rfic->CartesianFBpathSwitchtoforwardSummer = 0;
	d_rfic->set_reg_168();

	// Turn off Cartesian zero
	d_rfic->CartesianFBpathZeroEnable = 0;
	d_rfic->set_reg_170();

	// Turn off baseband feedback section
	// FIXME
	//d_rfic->tx_bb_fdbk_en = 0;
	//d_rfic->set_reg_157()

	// POR Off.  This enables the clock that drives the digital block (which provides the tap selection process).  It must be enabled to generate an output.  See Byp_fine, address 10, bit 6
	d_rfic->Clk_driver_en = 0;

	// POR Off
	d_rfic->qu_reg_en = 0;

	// POR Off
	d_rfic->qq_reg_en = 0;

	// POR Off
	d_rfic->win_rst = 0;

	// POR Off
	d_rfic->fineEn = 0;

	// POR Off
	d_rfic->fineEnb = 0;

	// POR Off
	//d_rfic->rsffEn = 0;

	// POR Off
	d_rfic->dl_en = 0;

	// POR Off
	d_rfic->cp_en = 0;

	d_rfic->set_reg_20();
	d_rfic->set_reg_21();

	// Put digital block in digital reset state
	d_rfic->Rst_n_async = 0;
	d_rfic->set_reg_24();
	shutdown();
	

}

bool
db_rfic_tx::select_tx_antenna(std::string which_antenna){
	//
	//Specify which antenna port to use for transmission.
	//@param which_antenna: either 'tx1', 'tx2' or 'tx3'
	//
	if (which_antenna == "tx1"){
		d_rfic->tx_output_channel_sel = 1;
		d_rfic->set_reg_172();
	}
	else if (which_antenna == "tx2"){
		d_rfic->tx_output_channel_sel = 2;
		d_rfic->set_reg_172();
	}
	else if (which_antenna  == "tx3"){
		d_rfic->tx_output_channel_sel = 4;
		d_rfic->set_reg_172();
	}
	else{
		std::cout<< "which_antenna must be either tx1/0 , tx2/1 or tx3/2"<<std::endl;
		return false;
	}
	
	return true;
}


bool
db_rfic_tx::select_tx_antenna(int which_antenna){
	//
	//Specify which antenna port to use for transmission.
	//@param which_antenna: either 'tx1', 'tx2' or 'tx3'
	//
        if (which_antenna == 0){
		d_rfic->tx_output_channel_sel = 1;
		d_rfic->set_reg_172();
	}
	else if (which_antenna == 1){
		d_rfic->tx_output_channel_sel = 2;
		d_rfic->set_reg_172();
	}
	else if (which_antenna  == 2){
		d_rfic->tx_output_channel_sel = 4;
		d_rfic->set_reg_172();
	}
	else{
		std::cout<< "which_antenna must be either tx1/0 , tx2/1 or tx3/2"<<std::endl;
		return false;
	}
	
	return true;
}

void
db_rfic_tx::shutdown()
{
  if (!d_is_shutdown){
    d_is_shutdown = true;
    shutdown_common();
  }
}

float
db_rfic_tx::gain_min()
{
  return 0.0;
}

float
db_rfic_tx::gain_max()
{
  return 45.0;
}

bool
db_rfic_tx::set_gain(float gain)
{
  return d_rfic->set_tx_gain(gain);
}

struct freq_result_t
db_rfic_tx::set_freq(double target_frequency)
{
  return d_rfic->set_tx_freq(target_frequency);
}

bool
db_rfic_tx::set_phase(int phase)
{
  return d_rfic->set_tx_phase(phase);
}

bool
db_rfic_tx::set_bw(float bw)
{
  return d_rfic->set_tx_bw(bw);
}

bool
db_rfic_tx::spectrum_inverted()
{
  return true;
}

float
db_rfic_tx::gain_db_per_step()
{
  return d_rfic->usrp()->pga_db_per_step();
}


db_rfic_rx::db_rfic_rx(usrp_basic_sptr usrp, int which)
  : db_rfic_base(usrp, which)
{
	std::string abc = "MIX5";
	this->select_rx_antenna(abc);
	d_rfic->set_reg_48();
	d_rfic->set_reg_49();
	d_rfic->set_reg_50();
	d_rfic->set_reg_51();
	d_rfic->set_reg_52();
	d_rfic->set_reg_53();
	d_rfic->set_reg_54();
	d_rfic->set_reg_55();
	d_rfic->set_reg_56();
	d_rfic->set_reg_57();
	d_rfic->set_reg_58();
	d_rfic->set_reg_60();
	d_rfic->set_reg_61();
	d_rfic->set_reg_62();
	d_rfic->set_reg_63();
	d_rfic->set_reg_64();
	d_rfic->set_reg_65();
	d_rfic->set_reg_66();
	d_rfic->set_reg_67();
	d_rfic->set_reg_68();
	d_rfic->set_reg_69();
	d_rfic->set_reg_70();
	d_rfic->set_reg_71();
	d_rfic->set_reg_72();
	d_rfic->set_reg_77();
	d_rfic->set_reg_78();
	d_rfic->set_reg_79();
	d_rfic->set_reg_80();
	d_rfic->set_reg_81();
	d_rfic->set_reg_82();
	d_rfic->set_reg_83();
	d_rfic->set_reg_84();
	d_rfic->set_reg_85();
	d_rfic->set_reg_86();
	d_rfic->set_reg_87();
	d_rfic->set_reg_88();
	d_rfic->set_reg_89();
	d_rfic->set_reg_90();
	d_rfic->set_reg_91();
	d_rfic->set_reg_96();
	d_rfic->set_reg_97();
	d_rfic->set_reg_98();
	d_rfic->set_reg_99();
	d_rfic->set_reg_104();
	d_rfic->set_reg_105();
	d_rfic->set_reg_106();
	d_rfic->set_reg_107();
	d_rfic->set_reg_108();
	d_rfic->set_reg_109();
	d_rfic->set_reg_110();
	d_rfic->set_reg_111();
	d_rfic->set_reg_112();
	d_rfic->set_reg_113();
	d_rfic->set_reg_114();
	d_rfic->set_reg_116();
	d_rfic->set_reg_117();
	d_rfic->set_reg_118();
	d_rfic->set_reg_119();
	d_rfic->set_reg_120();
	d_rfic->set_reg_121();
	d_rfic->set_reg_122();
	d_rfic->set_reg_123();
	d_rfic->set_reg_124();
	d_rfic->set_reg_125();
	d_rfic->set_reg_126();
	d_rfic->set_reg_127();
	d_rfic->set_reg_128();
	d_rfic->set_reg_133();
	d_rfic->set_reg_134();
	d_rfic->set_reg_135();
	d_rfic->set_reg_136();
	d_rfic->set_reg_137();
	d_rfic->set_reg_138();
	d_rfic->set_reg_139();
	d_rfic->set_reg_140();
	d_rfic->set_reg_141();
	d_rfic->set_reg_142();
	d_rfic->set_reg_143();
	d_rfic->set_reg_144();
	d_rfic->set_reg_145();
	d_rfic->set_reg_146();
	d_rfic->set_reg_147();
	d_rfic->set_reg_152();
	d_rfic->set_reg_153();
	d_rfic->set_reg_192();
	d_rfic->set_reg_193();
	d_rfic->set_reg_194();
	d_rfic->set_reg_195();
	d_rfic->set_reg_196();
	d_rfic->set_reg_197();
	d_rfic->set_reg_198();
	d_rfic->set_reg_199();
	d_rfic->set_reg_200();
	d_rfic->set_reg_201();
	d_rfic->set_reg_202();
	d_rfic->set_reg_203();
	d_rfic->set_reg_204();
	d_rfic->set_reg_205();
	d_rfic->set_reg_206();
	d_rfic->set_reg_207();

	// Get digital block out of digital reset state
	d_rfic->Rst_n_async_3 = 1;
	d_rfic->set_reg_128();

	// Set RX LNA port to LNA1 (SGO non-chopping mixer)
	// FIXME
	//d_rfic->rx_lna = 1;
	d_rfic->rx_lna = 5;

	// Set LNA bias
	d_rfic->rx_lnab = 1;

	// Enable LO clock to mixer
	d_rfic->rx_rxchen = 1;

	d_rfic->set_reg_205();

	// Enable RX Filter
	d_rfic->rx_fen = 1;

	// Enable baseband filter chopper clock
	d_rfic->rx_chcken = 1;

	// Enable chopper clock to all mixers
	d_rfic->rx_cen = 7;

	// Set chopper divide setting
	// FIXME
	//d_rfic->rx_chck = 0
	d_rfic->rx_chck = 1;

	d_rfic->set_reg_195();

	// Enable filter output
	d_rfic->rx_foe = 1;

	// Enable on-channel detector
	//d_rfic->rx_onchen = 1

	// Enable off-channel detector
	//d_rfic->rx_offchen = 1


	d_rfic->set_reg_196();

	// Set BQ filter Q to 1.33
	d_rfic->rx_qs = 2;

	// Set BQ resistor value to 1.4 kohms
	d_rfic->rx_rq = 0;

	d_rfic->set_reg_198();

	// Set VGA resistor value to 2.5 kohms
	d_rfic->rx_rv = 0;

	// Set PMA Rf resistor to 5 kohms
	d_rfic->rx_rfp = 00;

	d_rfic->set_reg_199();

	// Set compensation control
	d_rfic->rx_cc_2to0 = 0;

	d_rfic->set_reg_203();

	// Enable DCOC DAC
	d_rfic->rx_den = 1;

	d_rfic->set_reg_192();

	// Enable DCOC comparator
	d_rfic->rx_cmpen = 1;

	d_rfic->set_reg_193();

	// RC Tune enable
	// FIXME
	//d_rfic->rx_ten = 1;
	d_rfic->rx_ten = 0;

	// RC Tune ramp circuit enable
	// FIXME
	//d_rfic->rx_ren = 1;
	d_rfic->rx_ren = 0;

	// Select DCOC/RC Tune divider, divide by 8
	d_rfic->rx_dv = 3;

	d_rfic->set_reg_194();

	// Enable DCOC
	d_rfic->rx_dcoc = 1;

	d_rfic->set_reg_193();

	// POR On.  This enables the clock that drives the digital block (which provides the tap selection process).  It must be enabled to generate an output.  See Byp_fine, address 10, bit 6
	d_rfic->Clk_driver_en_3 = 1;

	// POR On
	d_rfic->qu_reg_en_3 = 1;

	// POR On
	d_rfic->qq_reg_en_3 = 1;
	// POR Off
	d_rfic->win_rst_3 = 0;

	// POR On
	d_rfic->fineEn_3 = 0 ;

	// POR Off
	d_rfic->fineEnb_3 = 1;

	// POR Off
	//d_rfic->rsffEn_3 = 0;

	// POR On
	d_rfic->dl_en_3 = 1;

	// POR On
	d_rfic->cp_en_3 = 1;

	d_rfic->set_reg_124();
	d_rfic->set_reg_125();
}

db_rfic_rx::~db_rfic_rx()
{
	// std::cout << "inside db_rfic_rx destructor "<< std::endl;
	// Power down

	// Set RX LNA path (off)
	d_rfic->rx_lna = 0;

	// Disable LO clock to mixer
	d_rfic->rx_rxchen = 0;

	d_rfic->set_reg_205();
	// Disable RX Filter
	d_rfic->rx_fen = 0;

	// Disable baseband filter chipper clock
	d_rfic->rx_chcken = 0;

	// Disable chopper clock to all mixers
	d_rfic->rx_cen = 0;
	d_rfic->set_reg_195();

	// Disable filter output
	d_rfic->rx_foe = 0;

	// Disable on-channel detector
	d_rfic->rx_onchen = 0;

	// Disable off-channel detector
	d_rfic->rx_offchen = 0;

	d_rfic->set_reg_196();

	// Disable DCOC DAC
	d_rfic->rx_den = 0;

	d_rfic->set_reg_192();

	// Disable DCOC comparator
	d_rfic->rx_cmpen = 0;

	d_rfic->set_reg_193();

	// RC Tune disable
	d_rfic->rx_ten = 0;

	// RC Tune ramp circuit disable
	d_rfic->rx_ren = 0;

	d_rfic->set_reg_194();

	// Disable DCOC
	d_rfic->rx_dcoc = 0;

	d_rfic->set_reg_193();

	// POR Off.  This enables the clock that drives the digital block (which provides the tap selection process).  It must be enabled to generate an output.  See Byp_fine, address 10, bit 6
	d_rfic->Clk_driver_en_3 = 0;

	// POR Off
	d_rfic->qu_reg_en_3 = 0;

	// POR Off
	d_rfic->qq_reg_en_3 = 0;

	// POR Off
	d_rfic->win_rst_3 = 0;

	// POR Off
	d_rfic->fineEn_3 = 0;

	// POR Off
	d_rfic->fineEnb_3 = 0;

	// POR Off
	//d_rfic->rsffEn_3 = 0;

	// POR Off
	d_rfic->dl_en_3 = 0;

	// POR Off
	d_rfic->cp_en_3 = 0;

	d_rfic->set_reg_124();
	d_rfic->set_reg_125();

	// Put digital block into digital reset state
	d_rfic->Rst_n_async_3 = 0;
	d_rfic->set_reg_58();
	shutdown();
	

}

void
db_rfic_rx::shutdown()
{
  if (!d_is_shutdown){
    d_is_shutdown = true;
    shutdown_common();
  }
}

bool
db_rfic_rx::select_rx_antenna(std::string which_antenna){
	//
	//Specif(y which antenna port to use for transmission.
	//@param which_antenna: either  LNA1/0, LNA2/1, LNA3/2, LNA4/3 or MIX5/4
	//
        //std::cout<<"inside select rx antenna..antenna used is :"<<which_antenna
	which_antenna = "MIX5";
	if( which_antenna == "LNA1"){
		d_rfic->rx_lna = 1;
		d_rfic->set_reg_205();
	}
	else if( which_antenna == "LNA2"){
		d_rfic->rx_lna = 2;
		d_rfic->set_reg_205();
	}
	else if( which_antenna == "LNA3"){
		d_rfic->rx_lna = 3;
		d_rfic->set_reg_205();
	}
	else if( which_antenna == "LNA4"){
		d_rfic->rx_lna = 4;
		d_rfic->set_reg_205();
	}
	else if( which_antenna == "MIX5"){
		d_rfic->rx_lna = 5;
		d_rfic->set_reg_205();
	}
	else{
		std::cout<< "FJAHFJDH which_antenna must be either LNA1/0, LNA2/1, LNA3/2, LNA4/3 or MIX5/4"<<std::endl;
		return false;
	}
	
	return true;
}

bool
db_rfic_rx::select_rx_antenna(int which_antenna){
	//
	//Specif(y which antenna port to use for transmission.
	//@param which_antenna: either 'rx1', 'rx2' or 'rx3'
	//
	if( which_antenna == 0){
		d_rfic->rx_lna = 1;
		d_rfic->set_reg_205();
	}
	else if( which_antenna == 1){
		d_rfic->rx_lna = 2;
		d_rfic->set_reg_205();
	}
	else if( which_antenna == 2){
		d_rfic->rx_lna = 3;
		d_rfic->set_reg_205();
	}
	else if( which_antenna == 3){
		d_rfic->rx_lna = 4;
		d_rfic->set_reg_205();
	}
	else if( which_antenna == 4){
		d_rfic->rx_lna = 5;
		d_rfic->set_reg_205();
	}
	else{
		std::cout<< "which_antenna must be either LNA1/0, LNA2/1, LNA3/2, LNA4/3 or MIX5/4"<<std::endl;
		return false;
	}
	return true;
}

float
db_rfic_rx::gain_min()
{
  return 0.0;
}

float
db_rfic_rx::gain_max()
{
  return 40.0;
}


bool
db_rfic_rx::set_gain(float gain)
{
  return d_rfic->set_rx_gain(gain);
}

struct freq_result_t
db_rfic_rx::set_freq(double target_frequency)
{
  return d_rfic->set_rx_freq(target_frequency);
}

bool
db_rfic_rx::set_phase(int phase)
{
  return d_rfic->set_rx_phase(phase);
}

bool
db_rfic_rx::set_bw(float bw)
{
  return d_rfic->set_rx_bw(bw);
}

void 
db_rfic_rx::enable_fb(){
	// Enable transmitter feedback to receiver for DC offset, etc.
	d_rfic->enable_tx_fb();
}

void 
db_rfic_rx::disable_tx_fb(){
	// Disable transmitter feedback to receiver
	return d_rfic->disable_tx_fb();
}
float 
db_rfic_rx::fb_gain_min(){
	return 0.0;
}

float 
db_rfic_rx::fb_gain_max(){
	return 40.0;
}

void 
db_rfic_rx::set_fb_gain(float gain){
	// Set feedback gain, in dB
	d_rfic->set_fb_gain(gain);
}

bool
db_rfic_rx::set_fb_freq(double target_freq){
	// Set feedback frequency, in Hz
	return d_rfic->set_fb_freq(target_freq);
}

bool 
db_rfic_rx::set_fb_phase(int phase){
	// Set feedback phase offset, in degrees
	return d_rfic->set_fb_phase(phase);
}

void 
db_rfic_rx::set_fb_bw(float bw){
	// Set feedback bandwidth, in Hz
	d_rfic->set_fb_bw(bw);
}

int 
db_rfic_rx::RSSI_fade(){
	// Get received signal strength indicators
	// Returns fade
	// Fade is proportional to how often the signal is low
	return d_rfic->RSSI_fade();
}

int 
db_rfic_rx::RSSI_clip(){
	// Get received signal strength indicators
	// Returns clip
	// Clip is proportional to how often the signal is high
	return d_rfic->RSSI_clip();
}

float
db_rfic_rx::gain_db_per_step()
{
  return d_rfic->usrp()->pga_db_per_step();
}

#endif


