use hal::i2c::peripheral::I2CEvent;
use hal::{
    gpio::{
        bank0::{Gpio12, Gpio13},
        FunctionI2c, Pin, PullUp,
    },
    pac,
};
use rp2040_hal as hal;
use toyfoc::ctrl::CtrlCMD;
use toyfoc::FOCStates;

type I2CPeripheral = rp2040_hal::i2c::peripheral::I2CPeripheralEventIterator<
    pac::I2C0,
    (
        Pin<Gpio12, FunctionI2c, PullUp>,
        Pin<Gpio13, FunctionI2c, PullUp>,
    ),
>;

pub struct I2Ctrl {
    pub i2c: I2CPeripheral,

    read_index: u8,
    pub cmd_id: u8,
    pub cmd: CtrlCMD,
    pub read_bytes: [u8; 4],

    pub data_id: u8,
}

impl I2Ctrl {
    pub fn new(i2c: I2CPeripheral) -> Self {
        I2Ctrl {
            i2c,

            cmd: CtrlCMD::None,

            cmd_id: 0,
            read_index: 0,
            read_bytes: [0u8; 4],

            data_id: 0,
        }
    }

    pub fn exchange(&mut self, foc_states: FOCStates) -> CtrlCMD {
        self.cmd = CtrlCMD::None;

        loop {
            let evt = self.i2c.next();

            match evt {
                None => {
                    return CtrlCMD::None;
                }
                Some(I2CEvent::TransferWrite) => {
                    let mut buf = [0u8; 5];
                    loop {
                        let r = self.i2c.read(&mut buf);

                        if r == 0 {
                            break;
                        }

                        for i in 0..r {
                            let byte = buf[i];

                            if self.read_index == 0 {
                                self.read_bytes = [0u8; 4];

                                if byte < 100 {
                                    self.cmd_id = byte;
                                } else if byte < 200 {
                                    self.data_id = byte;
                                }
                            } else if self.read_index < 5 {
                                self.read_bytes[(self.read_index - 1) as usize] = byte;
                            }
                            self.read_index += 1;
                        }
                    }
                }
                Some(I2CEvent::TransferRead) => {
                    self.write(foc_states);
                }
                Some(I2CEvent::Start | I2CEvent::Restart) => {
                    self.read_index = 0;
                    self.read_bytes = [0u8; 4];
                }
                Some(I2CEvent::Stop) => {
                    return self.read();
                }
            }
        }
    }

    pub fn read(&mut self) -> CtrlCMD {
        if self.cmd_id > 0 {
            let cmd_val = f32::from_le_bytes(self.read_bytes);
            self.cmd = CtrlCMD::new(self.cmd_id, cmd_val);
            self.cmd_id = 0;
        }

        self.cmd
    }

    pub fn write(&mut self, foc_states: FOCStates) {
        let (mut first, mut second, mut third) = (0.0, 0.0, 0.0);

        if self.data_id == foc_states.conf_base.id() {
            (first, second, third) = foc_states.conf_base.value();
        } else if self.data_id == foc_states.conf_velocity.id() {
            (first, second, third) = foc_states.conf_velocity.value();
        } else if self.data_id == foc_states.conf_position.id() {
            (first, second, third) = foc_states.conf_velocity.value();
        } else if self.data_id == foc_states.conf_torque.id() {
            (first, second, third) = foc_states.conf_torque.value();
        } else if self.data_id == foc_states.conf_velocity_pid.id() {
            (first, second, third) = foc_states.conf_velocity_pid.value();
        } else if self.data_id == foc_states.conf_position_pid.id() {
            (first, second, third) = foc_states.conf_position_pid.value();
        } else if self.data_id == foc_states.conf_torque_pid.id() {
            (first, second, third) = foc_states.conf_torque_pid.value();
        } else if self.data_id == foc_states.conf_limit.id() {
            (first, second, third) = foc_states.conf_limit.value();
        } else if self.data_id == foc_states.conf_voltage_offset.id() {
            (first, second, third) = foc_states.conf_voltage_offset.value();
        } else if self.data_id == foc_states.data_base.id() {
            (first, second, third) = foc_states.data_base.value();
        } else if self.data_id == foc_states.data_q.id() {
            (first, second, third) = foc_states.data_q.value();
        } else if self.data_id == foc_states.data_current.id() {
            (first, second, third) = foc_states.data_current.value();
        } else if self.data_id == foc_states.data_time.id() {
            (first, second, third) = foc_states.data_time.value();
        }

        let mut buf = [0u8; 13];
        buf[0] = self.data_id;

        buf[1..5].copy_from_slice(&(first.to_le_bytes()));
        buf[5..9].copy_from_slice(&(second.to_le_bytes()));
        buf[9..13].copy_from_slice(&(third.to_le_bytes()));
        self.i2c.write(&buf);
    }
}
