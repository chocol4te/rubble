//! Link-Layer connection management and LLCP implementation.

use {
    crate::{
        bytes::*,
        link::{
            advertising::ConnectRequestData,
            data::{self, ControlPdu, Header, Llid, Pdu},
            queue::{Consume, Consumer, Producer},
            Cmd, CompanyId, FeatureSet, HardwareInterface, NextUpdate, RadioCmd, SeqNum,
            Transmitter,
        },
        phy::{ChannelMap, DataChannel},
        time::{Duration, Instant, Timer},
        utils::{Hex, HexSlice},
        BLUETOOTH_VERSION,
    },
    core::{marker::PhantomData, num::Wrapping},
    log::{info, trace},
};

/// Connection state.
pub struct Connection<HW: HardwareInterface> {
    access_address: u32,
    crc_init: u32,
    channel_map: ChannelMap,

    /// Number of (unmapped) channels to hop between each connection event.
    hop: u8,

    /// Connection event interval (duration between the start of 2 subsequent connection events).
    conn_interval: Duration,

    /// Connection event counter (`connEventCount`).
    conn_event_count: Wrapping<u16>,

    /// Unmapped data channel on which the next connection event will take place.
    ///
    /// Also known as `lastUnmappedChannel` or `previous_event_channel` (yes, the spec uses both).
    unmapped_channel: DataChannel,

    /// Actual data channel on which the next data packets will be exchanged.
    channel: DataChannel,

    // Acknowledgement / Flow Control state
    /// `SN` bit to be used
    transmit_seq_num: SeqNum,
    next_expected_seq_num: SeqNum,

    /// Header of the last transmitted packet, used for retransmission.
    last_header: data::Header,

    /// Whether we have ever received a data packet in this connection.
    received_packet: bool,

    tx: Consumer,
    rx: Producer,

    _p: PhantomData<HW>,
}

impl<HW: HardwareInterface> Connection<HW> {
    /// Initializes a connection state according to the `LLData` contained in the `CONNECT_REQ`
    /// advertising PDU.
    ///
    /// Returns the connection state and a `Cmd` to apply to the radio/timer.
    ///
    /// # Parameters
    ///
    /// * **`lldata`**: Data contained in the `CONNECT_REQ` advertising PDU.
    /// * **`rx_end`**: Instant at which the `CONNECT_REQ` PDU was fully received.
    /// * **`tx`**: Channel for packets to transmit.
    /// * **`rx`**: Channel for received packets.
    pub fn create(
        lldata: &ConnectRequestData,
        rx_end: Instant,
        tx: Consumer,
        rx: Producer,
    ) -> (Self, Cmd) {
        assert_eq!(
            lldata.slave_latency(),
            0,
            "slave latency is not implemented"
        );

        let mut this = Self {
            access_address: lldata.access_address(),
            crc_init: lldata.crc_init().into(),
            channel_map: *lldata.channel_map(),
            hop: lldata.hop(),
            conn_interval: lldata.interval(),
            conn_event_count: Wrapping(0xFFFF), // wraps to 0 on first event

            unmapped_channel: DataChannel::new(0),
            channel: DataChannel::new(0),

            transmit_seq_num: SeqNum::ZERO,
            next_expected_seq_num: SeqNum::ZERO,
            last_header: Header::new(Llid::DataCont),
            received_packet: false,

            tx,
            rx,

            _p: PhantomData,
        };

        // Calculate the first channel to use
        this.hop_channel();

        let cmd = Cmd {
            next_update: NextUpdate::At(
                rx_end + lldata.end_of_tx_window() + Duration::from_micros(500),
            ),
            radio: RadioCmd::ListenData {
                channel: this.channel,
                access_address: this.access_address,
                crc_init: this.crc_init,
            },
        };

        (this, cmd)
    }

    /// Called by the `LinkLayer` when a data channel packet is received.
    ///
    /// Returns `Err(())` when the connection is ended (not necessarily due to an error condition).
    pub fn process_data_packet(
        &mut self,
        _rx_end: Instant,
        tx: &mut HW::Tx,
        timer: &mut HW::Timer,
        header: data::Header,
        payload: &[u8],
        crc_ok: bool,
    ) -> Result<Cmd, ()> {
        // If the sequence number of the packet is the same as our next expected sequence number,
        // the packet contains new data that we should try to process. However, if the CRC is bad,
        // we'll never try to process the data and instead request a retransmission.
        let is_new = header.sn() == self.next_expected_seq_num && crc_ok;

        // If the packet's "NESN" is equal to our last sent sequence number + 1, the other side has
        // acknowledged our last packet (and is now expecting one with an incremented seq. num.).
        // However, if the CRC is bad, the bit might be flipped, so we cannot assume that the packet
        // was acknowledged and thus always retransmit.
        let acknowledged = header.nesn() == self.transmit_seq_num + SeqNum::ONE && crc_ok;

        let is_empty = header.llid() == Llid::DataCont && payload.is_empty();

        if is_new {
            if is_empty {
                // Always acknowledge empty packets, no need to process them
                self.next_expected_seq_num += SeqNum::ONE;
            } else if header.llid() == Llid::Control {
                // LLCP message, process it immediately, falling back to using the channel if this
                // is impossible
                if let Ok(pdu) = ControlPdu::from_bytes(&mut ByteReader::new(payload)) {
                    self.process_control_pdu(pdu);
                }
            } else {
                // Try to buffer the packet. If it fails, we don't acknowledge it, so it will be
                // resent until we have space.

                if self.rx.produce_raw(header, payload).is_ok() {
                    // Acknowledge the packet
                    self.next_expected_seq_num += SeqNum::ONE;
                } else {
                    trace!("NACK (no space in rx buffer)");
                }
            }
        }

        if acknowledged {
            self.received_packet = true;
            // Here we'll always send a new packet (which might be empty if we don't have anything
            // to say). If `needs_processing` is set, we'll also process the received PDU before
            // sending.

            self.transmit_seq_num += SeqNum::ONE;

            // Send a new packet.

            // Write payload data. Try to acquire PDU from the tx queue, fall back to an empty PDU.
            let mut payload_writer = ByteWriter::new(tx.tx_payload_buf());
            let header = match self.tx.consume_raw_with(|header, pl| {
                payload_writer.write_slice(pl).expect("TX buf out of space");
                Consume::always(Ok(header))
            }) {
                Ok(h) => h,
                Err(_) => Header::new(Llid::DataCont),
            };

            self.send(header, tx);
        } else {
            // Last packet not acknowledged, resend.
            // If CRC is bad, this bit could be flipped, so we always retransmit in that case.
            if self.received_packet {
                self.last_header.set_nesn(self.next_expected_seq_num);
                tx.transmit_data(
                    self.access_address,
                    self.crc_init,
                    self.last_header,
                    self.channel,
                );
                trace!("<<RESENT>>");
            } else {
                // We've never received (and thus sent) a data packet before, so we can't
                // *re*transmit anything. Send empty PDU instead.
                // (this should not really happen, though!)
                self.received_packet = true;

                let pdu = Pdu::empty();
                let mut payload_writer = ByteWriter::new(tx.tx_payload_buf());
                pdu.to_bytes(&mut payload_writer).unwrap();
                self.send(Header::new(pdu.llid()), tx);
            }
        }

        let last_channel = self.channel;

        // FIXME: Don't hop if one of the MD bits is set to true
        {
            // Connection event closes
            self.hop_channel();
            self.conn_event_count += Wrapping(1);
        }

        trace!(
            "#{} DATA({}->{})<- {}{:?}, {:?}",
            self.conn_event_count,
            last_channel.index(),
            self.channel.index(),
            if crc_ok { "" } else { "BADCRC, " },
            header,
            HexSlice(payload)
        );

        Ok(Cmd {
            next_update: NextUpdate::At(timer.now() + self.conn_event_timeout()),
            radio: RadioCmd::ListenData {
                channel: self.channel,
                access_address: self.access_address,
                crc_init: self.crc_init,
            },
        })
    }

    /// Must be called when the configured timer expires (according to a `Cmd` returned earlier).
    pub fn timer_update(&mut self, timer: &mut HW::Timer) -> Result<Cmd, ()> {
        if self.received_packet {
            // No packet from master, skip this connection event and listen on the next channel

            let last_channel = self.channel;
            self.hop_channel();
            self.conn_event_count += Wrapping(1);
            trace!(
                "DATA({}->{}): missed conn event #{}",
                last_channel.index(),
                self.channel.index(),
                self.conn_event_count.0,
            );

            Ok(Cmd {
                next_update: NextUpdate::At(timer.now() + self.conn_event_timeout()),
                radio: RadioCmd::ListenData {
                    channel: self.channel,
                    access_address: self.access_address,
                    crc_init: self.crc_init,
                },
            })
        } else {
            // Master did not transmit the first packet during this transmit window.

            // TODO: Move the transmit window forward by the `connInterval`.

            self.conn_event_count += Wrapping(1);
            trace!("missed transmit window");
            Err(())
        }
    }

    fn conn_event_timeout(&self) -> Duration {
        // Time out ~500µs after the anchor point of the next conn event.
        self.conn_interval + Duration::from_micros(500)
    }

    /// Whether we want to send more data during this connection event.
    ///
    /// Note that this *has to* change to `false` eventually, even if there's more data to be sent,
    /// because the connection event must close at least `T_IFS` before the next one occurs.
    fn has_more_data(&self) -> bool {
        false
    }

    /// Advances the `unmapped_channel` and `channel` fields to the next data channel on which a
    /// connection event will take place.
    ///
    /// According to: `4.5.8.2 Channel Selection`.
    fn hop_channel(&mut self) {
        let unmapped_channel = DataChannel::new((self.unmapped_channel.index() + self.hop) % 37);

        self.unmapped_channel = unmapped_channel;
        self.channel = if self.channel_map.is_used(unmapped_channel) {
            unmapped_channel
        } else {
            // This channel isn't used, remap channel according to map
            let remapping_index = unmapped_channel.index() % self.channel_map.num_used_channels();
            self.channel_map.by_index(remapping_index)
        };
    }

    /// Sends a new PDU to the connected device (ie. a non-retransmitted PDU).
    fn send(&mut self, mut header: Header, tx: &mut HW::Tx) {
        header.set_md(self.has_more_data());
        header.set_nesn(self.next_expected_seq_num);
        header.set_sn(self.transmit_seq_num);
        self.last_header = header;

        tx.transmit_data(self.access_address, self.crc_init, header, self.channel);

        let pl = &tx.tx_payload_buf()[..usize::from(header.payload_length())];
        trace!("DATA->{:?}, {:?}", header, HexSlice(pl));
    }

    /// Tries to process and acknowledge an LL Control PDU.
    fn process_control_pdu(&mut self, pdu: ControlPdu) {
        // Acknowledge PDU
        self.next_expected_seq_num += SeqNum::ONE;

        info!("<- LL Control PDU: {:?}", pdu);
        let response = match pdu {
            ControlPdu::FeatureReq { features_master } => ControlPdu::FeatureRsp {
                features_used: features_master & FeatureSet::supported(),
            },
            ControlPdu::VersionInd { .. } => {
                // FIXME this should be something real, and defined somewhere else
                let comp_id = 0xFFFF;
                // FIXME this should correlate with the Cargo package version
                let sub_vers_nr = 0x0000;

                ControlPdu::VersionInd {
                    vers_nr: BLUETOOTH_VERSION,
                    comp_id: CompanyId::from_raw(comp_id),
                    sub_vers_nr: Hex(sub_vers_nr),
                }
            }
            _ => ControlPdu::UnknownRsp {
                unknown_type: pdu.opcode(),
            },
        };
        info!("-> Response: {:?}", response);
    }
}
