# This script will create a standalone domain for a processor and its architecture
# Workspace should be set externally

set script [info script] 
set script_dir [file normalize [file dirname $script]]

puts "INFO: Running $script"

set domain_name [file tail $script_dir]

# Modify these for custom domain/BSP settings
set arch "32-bit"
set os "standalone"
set proc "microblaze_0"

# Destination platform needs to be made active first
platform active "hw_platform"

domain create -name $domain_name -proc $proc -arch $arch -os $os

# Customize BSP, this replaces *.mss file
bsp config clocking "false"
bsp config enable_sw_intrusive_profiling "false"
bsp config hypervisor_guest "false"
bsp config lockstep_mode_debug "false"
bsp config microblaze_exceptions "false"
bsp config predecode_fpu_exceptions "false"
bsp config profile_timer "none"
bsp config sleep_timer "none"
bsp config stdin "axi_uartlite_0"
bsp config stdout "axi_uartlite_0"
bsp config ttc_select_cntr "2"
bsp config zynqmp_fsbl_bsp "false"
bsp config archiver "mb-ar"
bsp config assembler "mb-as"
bsp config compiler "mb-gcc"
bsp config compiler_flags "-O2 -c"
bsp config extra_compiler_flags "-g -O0 -ffunction-sections -fdata-sections -Wall -Wextra"
