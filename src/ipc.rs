#[derive(Debug)]
pub enum IPC {
    DisableMotors,
    EnableMotors,
    SetCtrlMode(copter_defs::CtrlMode),
}
