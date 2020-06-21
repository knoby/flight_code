#[derive(Debug)]
pub enum IPC {
    ArmMotors,
    EnableMotors,
    SetCtrlMode(CtrlMode),
}

#[derive(Debug)]
pub enum CtrlMode {
    /// Direct Control of the Motor speed over the serial communication
    DirectCtrl(f16, f16, f16, f16),
    /// Control of the Yaw, Roll and Pitch angle with a two controll cascades
    AngleCtrl(f16, f16, f16),
}
