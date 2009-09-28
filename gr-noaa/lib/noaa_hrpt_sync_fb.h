/* -*- c++ -*- */
/*
 * Copyright 2009 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_NOAA_HRPT_SYNC_FB_H
#define INCLUDED_NOAA_HRPT_SYNC_FB_H

#include <gr_block.h>

class noaa_hrpt_sync_fb;
typedef boost::shared_ptr<noaa_hrpt_sync_fb> noaa_hrpt_sync_fb_sptr;

noaa_hrpt_sync_fb_sptr
noaa_make_hrpt_sync_fb(float alpha, float beta, float sps, float max_offset);

class noaa_hrpt_sync_fb : public gr_block
{
  friend noaa_hrpt_sync_fb_sptr noaa_make_hrpt_sync_fb(float alpha, float beta, float sps, float max_offset);
  noaa_hrpt_sync_fb(float alpha, float beta, float sps, float max_offset);

  float d_alpha;		// 1st order loop constant
  float d_beta;			// 2nd order loop constant
  float d_sps;                  // samples per symbol
  float d_max_offset;		// Maximum frequency offset for d_sps, samples/symbol
  float d_phase;		// Instantaneous symbol phase
  float d_freq;			// Instantaneous symbol frequency, samples/symbol
  int   d_last_sign;            // Tracks zero crossings

 public:
  int general_work(int noutput_items,
		   gr_vector_int &ninput_items,
		   gr_vector_const_void_star &input_items,
		   gr_vector_void_star &output_items);

  void set_alpha(float alpha) { d_alpha = alpha; }
  void set_beta(float beta) { d_beta = beta; }
  void set_max_offset(float max_offset) { d_max_offset = max_offset; }
};

#endif /* INCLUDED_NOAA_HRPT_SYNC_FB_H */
