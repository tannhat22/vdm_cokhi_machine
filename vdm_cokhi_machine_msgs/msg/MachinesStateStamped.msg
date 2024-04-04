uint8 DAY_SHIFT = 0
uint8 NIGHT_SHIFT = 1

std_msgs/Header header 

int32 shift
int32 machines_quantity
int32 types_quantity
int32[] id_machines
MachineState[] state_machines
OverralMachine[] overral_machines