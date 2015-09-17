/* -*- c++ -*- */
/*
 * Copyright 2010-2015 Free Software Foundation, Inc.
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

#include <climits>
#include <stdexcept>
#include "usrp_msg_sink_impl.h"
#include "gr_uhd_common.h"
#include <gnuradio/io_signature.h>

namespace gr {
  namespace uhd {

    static const size_t ALL_CHANS = ::uhd::usrp::multi_usrp::ALL_CHANS;

    usrp_msg_sink::sptr
    usrp_msg_sink::make(const ::uhd::device_addr_t &device_addr,
                    const ::uhd::stream_args_t &stream_args,
                    const std::string &length_tag_name)
    {
      check_abi();
      return usrp_msg_sink::sptr
        (new usrp_msg_sink_impl(device_addr, stream_args_ensure(stream_args), length_tag_name));
    }

    usrp_msg_sink_impl::usrp_msg_sink_impl(const ::uhd::device_addr_t &device_addr,
                                   const ::uhd::stream_args_t &stream_args,
                                   const std::string &length_tag_name)
      : usrp_block("usrp_msg_sink",
                      args_to_io_sig(stream_args),
                      io_signature::make(0, 0, 0)),
        usrp_block_impl(device_addr, stream_args, length_tag_name),
        _length_tag_key(length_tag_name.empty() ? pmt::PMT_NIL : pmt::string_to_symbol(length_tag_name)),
        _nitems_to_send(0),
        d_finished(false),
        d_index_start(0),
        d_remaining(0),
        d_data_msgq(pmt::mp("message_in"))
    {
      _sample_rate = get_samp_rate();
	d_thread = boost::shared_ptr<boost::thread>(
	        new boost::thread(boost::bind(&usrp_msg_sink_impl::run, this)));
	message_port_register_in(d_data_msgq);
    }

    usrp_msg_sink_impl::~usrp_msg_sink_impl()
    {
	    d_finished=true;
    }

    ::uhd::dict<std::string, std::string>
    usrp_msg_sink_impl::get_usrp_info(size_t chan)
    {
      chan = _stream_args.channels[chan];
#ifdef UHD_USRP_MULTI_USRP_GET_USRP_INFO_API
      return _dev->get_usrp_tx_info(chan);
#else
      throw std::runtime_error("not implemented in this version");
#endif
    }

    void
    usrp_msg_sink_impl::set_subdev_spec(const std::string &spec,
                                    size_t mboard)
    {
      return _dev->set_tx_subdev_spec(spec, mboard);
    }

    std::string
    usrp_msg_sink_impl::get_subdev_spec(size_t mboard)
    {
      return _dev->get_tx_subdev_spec(mboard).to_string();
    }

    void
    usrp_msg_sink_impl::set_samp_rate(double rate)
    {
        BOOST_FOREACH(const size_t chan, _stream_args.channels)
        {
            _dev->set_tx_rate(rate, chan);
        }
      _sample_rate = this->get_samp_rate();
    }

    double
    usrp_msg_sink_impl::get_samp_rate(void)
    {
      return _dev->get_tx_rate(_stream_args.channels[0]);
    }

    ::uhd::meta_range_t
    usrp_msg_sink_impl::get_samp_rates(void)
    {
#ifdef UHD_USRP_MULTI_USRP_GET_RATES_API
      return _dev->get_tx_rates(_stream_args.channels[0]);
#else
      throw std::runtime_error("not implemented in this version");
#endif
    }

    ::uhd::tune_result_t
    usrp_msg_sink_impl::set_center_freq(const ::uhd::tune_request_t tune_request,
                                    size_t chan)
    {
      _curr_tune_req[chan] = tune_request;
      chan = _stream_args.channels[chan];
      return _dev->set_tx_freq(tune_request, chan);
    }

    SET_CENTER_FREQ_FROM_INTERNALS(usrp_msg_sink_impl, set_tx_freq);

    double
    usrp_msg_sink_impl::get_center_freq(size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->get_tx_freq(chan);
    }

    ::uhd::freq_range_t
    usrp_msg_sink_impl::get_freq_range(size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->get_tx_freq_range(chan);
    }

    void
    usrp_msg_sink_impl::set_gain(double gain, size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->set_tx_gain(gain, chan);
    }

    void
    usrp_msg_sink_impl::set_gain(double gain,
                             const std::string &name,
                             size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->set_tx_gain(gain, name, chan);
    }

    void usrp_msg_sink_impl::set_normalized_gain(double norm_gain, size_t chan)
    {
      if (norm_gain > 1.0 || norm_gain < 0.0) {
        throw std::runtime_error("Normalized gain out of range, must be in [0, 1].");
      }
      ::uhd::gain_range_t gain_range = get_gain_range(chan);
      double abs_gain = (norm_gain * (gain_range.stop() - gain_range.start())) + gain_range.start();
      set_gain(abs_gain, chan);
    }

    double
    usrp_msg_sink_impl::get_gain(size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->get_tx_gain(chan);
    }

    double
    usrp_msg_sink_impl::get_gain(const std::string &name, size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->get_tx_gain(name, chan);
    }

    double
    usrp_msg_sink_impl::get_normalized_gain(size_t chan)
    {
      ::uhd::gain_range_t gain_range = get_gain_range(chan);
      double norm_gain =
        (get_gain(chan) - gain_range.start()) /
        (gain_range.stop() - gain_range.start());
      // Avoid rounding errors:
      if (norm_gain > 1.0) return 1.0;
      if (norm_gain < 0.0) return 0.0;
      return norm_gain;
    }

    std::vector<std::string>
    usrp_msg_sink_impl::get_gain_names(size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->get_tx_gain_names(chan);
    }

    ::uhd::gain_range_t
    usrp_msg_sink_impl::get_gain_range(size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->get_tx_gain_range(chan);
    }

    ::uhd::gain_range_t
    usrp_msg_sink_impl::get_gain_range(const std::string &name,
                                   size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->get_tx_gain_range(name, chan);
    }

    void
    usrp_msg_sink_impl::set_antenna(const std::string &ant,
                                size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->set_tx_antenna(ant, chan);
    }

    std::string
    usrp_msg_sink_impl::get_antenna(size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->get_tx_antenna(chan);
    }

    std::vector<std::string>
    usrp_msg_sink_impl::get_antennas(size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->get_tx_antennas(chan);
    }

    void
    usrp_msg_sink_impl::set_bandwidth(double bandwidth, size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->set_tx_bandwidth(bandwidth, chan);
    }

    double
    usrp_msg_sink_impl::get_bandwidth(size_t chan)
    {
        chan = _stream_args.channels[chan];
        return _dev->get_tx_bandwidth(chan);
    }

    ::uhd::freq_range_t
    usrp_msg_sink_impl::get_bandwidth_range(size_t chan)
    {
        chan = _stream_args.channels[chan];
        return _dev->get_tx_bandwidth_range(chan);
    }

    void
    usrp_msg_sink_impl::set_dc_offset(const std::complex<double> &offset,
                                  size_t chan)
    {
      chan = _stream_args.channels[chan];
#ifdef UHD_USRP_MULTI_USRP_FRONTEND_CAL_API
      return _dev->set_tx_dc_offset(offset, chan);
#else
      throw std::runtime_error("not implemented in this version");
#endif
    }

    void
    usrp_msg_sink_impl::set_iq_balance(const std::complex<double> &correction,
                                   size_t chan)
    {
      chan = _stream_args.channels[chan];
#ifdef UHD_USRP_MULTI_USRP_FRONTEND_CAL_API
      return _dev->set_tx_iq_balance(correction, chan);
#else
      throw std::runtime_error("not implemented in this version");
#endif
    }

    ::uhd::sensor_value_t
    usrp_msg_sink_impl::get_sensor(const std::string &name, size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->get_tx_sensor(name, chan);
    }

    std::vector<std::string>
    usrp_msg_sink_impl::get_sensor_names(size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->get_tx_sensor_names(chan);
    }

    ::uhd::usrp::dboard_iface::sptr
    usrp_msg_sink_impl::get_dboard_iface(size_t chan)
    {
      chan = _stream_args.channels[chan];
      return _dev->get_tx_dboard_iface(chan);
    }

    void
    usrp_msg_sink_impl::set_stream_args(const ::uhd::stream_args_t &stream_args)
    {
      _update_stream_args(stream_args);
#ifdef GR_UHD_USE_STREAM_API
      _tx_stream.reset();
#else
      throw std::runtime_error("not implemented in this version");
#endif
    }

    /***********************************************************************
     * Work
     **********************************************************************/
void
usrp_msg_sink_impl::run()
{

	while (!d_finished) {
		_metadata.has_time_spec=false;
		gr_vector_const_void_star input_items;

		d_pmt_tuple = delete_head_blocking(d_data_msgq, 0);
		d_remaining = pmt::to_uint64(
		        pmt::tuple_ref(d_pmt_tuple, 0));
		uint8_t* items = (uint8_t *) pmt::blob_data(pmt::tuple_ref(d_pmt_tuple, 1));
		input_items.push_back(items);
		_metadata.start_of_burst=true;
		_metadata.end_of_burst=true;

		int ninput_items=d_remaining;

#ifdef GR_UHD_USE_STREAM_API
		//send all ninput_items with metadata
		const size_t num_sent = _tx_stream->send(input_items,
		                                         ninput_items,
		                                         _metadata, 1.0);

#else
		const size_t num_sent = _dev->get_device()->send
		(input_items, ninput_items, _metadata,
			*_type, ::uhd::device::SEND_MODE_FULL_BUFF, 1.0);
#endif


	}
}

int
usrp_msg_sink_impl::
work(int noutput_items, gr_vector_const_void_star &input_items,
     gr_vector_void_star &output_items)
{
	return 0;
}



    void
    usrp_msg_sink_impl::set_start_time(const ::uhd::time_spec_t &time)
    {
      _start_time = time;
      _start_time_set = true;
      _stream_now = false;
    }

    //Send an empty start-of-burst packet to begin streaming.
    //Set at a time in the near future to avoid late packets.
    bool
    usrp_msg_sink_impl::start(void)
    {
#ifdef GR_UHD_USE_STREAM_API
      _tx_stream = _dev->get_tx_stream(_stream_args);
#endif

      _metadata.start_of_burst = true;
      _metadata.end_of_burst = false;
      // Bursty tx will need to send a tx_time to activate time spec
      _metadata.has_time_spec = !_stream_now && pmt::is_null(_length_tag_key);
      _nitems_to_send = 0;
      if(_start_time_set) {
        _start_time_set = false; //cleared for next run
        _metadata.time_spec = _start_time;
      }
      else {
        _metadata.time_spec = get_time_now() + ::uhd::time_spec_t(0.15);
      }

#ifdef GR_UHD_USE_STREAM_API
      _tx_stream->send
        (gr_vector_const_void_star(_nchan), 0, _metadata, 1.0);
#else
      _dev->get_device()->send
        (gr_vector_const_void_star(_nchan), 0, _metadata,
         *_type, ::uhd::device::SEND_MODE_ONE_PACKET, 1.0);
#endif
      return true;
    }

    //Send an empty end-of-burst packet to end streaming.
    //Ending the burst avoids an underflow error on stop.
    bool
    usrp_msg_sink_impl::stop(void)
    {
      _metadata.start_of_burst = false;
      _metadata.end_of_burst = true;
      _metadata.has_time_spec = false;
      _nitems_to_send = 0;

#ifdef GR_UHD_USE_STREAM_API
      _tx_stream->send(gr_vector_const_void_star(_nchan), 0, _metadata, 1.0);
#else
      _dev->get_device()->send
        (gr_vector_const_void_star(_nchan), 0, _metadata,
         *_type, ::uhd::device::SEND_MODE_ONE_PACKET, 1.0);
#endif
      return true;
    }

  } /* namespace uhd */
} /* namespace gr */
