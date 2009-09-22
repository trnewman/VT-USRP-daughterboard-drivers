#ifndef DB_rfic_H
#define DB_rfic_H

#include <db_base.h>
#include <boost/shared_ptr.hpp>

class rfic;
typedef boost::shared_ptr<rfic> rfic_sptr;


/******************************************************************************/

class db_rfic_base: public db_base
{
  /*
   * Abstract base class for all rfic boards.
   * 
   * Derive board specific subclasses from db_rfic_base_{tx,rx}
   */
public:
  db_rfic_base(usrp_basic_sptr usrp, int which);
  ~db_rfic_base();
  bool is_quadrature();
  double freq_min();
  double freq_max();
  
protected:
  rfic_sptr d_rfic;
  void shutdown_common();
};


/******************************************************************************/


class db_rfic_tx : public db_rfic_base
{
protected:
  void shutdown();

public:
  db_rfic_tx(usrp_basic_sptr usrp, int which);
  ~db_rfic_tx();

  float gain_min();
  float gain_max();
  bool select_tx_antenna(std::string which_antenna);
  bool select_tx_antenna(int which_antenna);
  struct freq_result_t set_freq(double target_frequency);
  bool set_gain(float gain);
  bool set_phase(int phase);
  bool set_bw(float bw);
  bool spectrum_inverted();
  float gain_db_per_step();
  
};

class db_rfic_rx : public db_rfic_base
{
protected:
  void shutdown();

public:
  db_rfic_rx(usrp_basic_sptr usrp, int which);
  ~db_rfic_rx();

  float gain_min();
  float gain_max();
  bool select_rx_antenna(std::string which_antenna);
  bool select_rx_antenna(int which_antenna);
  struct freq_result_t set_freq(double target_frequency);
  bool set_gain(float gain);
  bool set_phase(int phase);
  bool set_bw(float bw);
  void enable_fb();
  void disable_tx_fb();
  float fb_gain_min();
  float fb_gain_max();
  void set_fb_gain(float gain);
  bool set_fb_freq(double target_freq);
  bool set_fb_phase(int phase);
  void set_fb_bw(float bw);  
  int RSSI_fade();
  int RSSI_clip();
  float gain_db_per_step();

};




#endif
