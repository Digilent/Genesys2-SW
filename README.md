# Genesys2 Software Repository

This repository contains Vitis workspaces for all software demos for the Genesys2.

For more information about the Genesys2, visit its [Resource Center](https://reference.digilentinc.com/reference/programmable-logic/genesys-2/start) on the Digilent Wiki.

Each demo contained in this repository is documented on the Digilent Wiki, links in the table below.

| Name and Wiki Link | Description |
|--------------------|-------------|
| [Genesys2 FMC Pcam Adapter](https://reference.digilentinc.com/reference/programmable-logic/genesys-2/demos/genesys-2_fmc-pcam-adapter_demo) | Demonstrates the usage of the FMC Pcam Adapter as an interface from one up to four different Pcam cameras and the Genesys2 platform. |

## Repository Description

This repository contains the Vitis workspace and software sources for all of the software demos that we provide for the Genesys2. As each of these demos also requires a hardware design contained in a Vivado project, this repository should not be used directly. The [Genesys2](https://github.com/Digilent/Genesys2) repository contains all sources for these demos across all tools, and pulls in all of this repository's sources by using it as a submodule.

For instructions on how to use this repository with git, and for additional documentation on the submodule and branch structures used, please visit [Digilent FPGA Demo Git Repositories](https://reference.digilentinc.com/reference/programmable-logic/documents/git) on the Digilent Wiki. Note that use of git is not required to use this demo. Digilent recommends the use of project releases, for which instructions can be found in each demo wiki page, linked in the table of demos, above.

Demos were moved into this repository during 2020.1 updates. History of these demos prior to these updates can be found in their old repositories, linked below:

