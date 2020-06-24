#[derive(Debug)]
pub enum IPC {
    DisableMotors,
    EnableMotors,
    SetCtrlMode(CtrlMode),
}

#[derive(Debug)]
pub enum CtrlMode {
    /// Direct Control of the Motor speed over the serial communication
    DirectCtrl([f32; 4]),
    /// Control of the Yaw, Roll and Pitch angle with a two controll cascades
    AngleCtrl(f32, f32, f32),
}
