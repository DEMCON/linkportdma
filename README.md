# ADSP-SC589 linkport driver using DMA

This driver improves the Blackfin/ADSP-SC58x linkport driver, as developed by
Analog Devices, by utilizing DMA and interrupts.  It is a high-performance
high-bandwidth non-blocking double-buffered driver. It can be used using normal
`read()` and `write()` calls, but to use the double buffering efficiently, more
complex `ioctl()` should be used to indicate how much data is expected.

`bfin_lp.c` replaces the `buildroot/linux/linux-kernel/drivers/char/bfin_lp.c`,
as distributed within the CCES Linux add-in 1.1.0, with buildroot-sc58x-1.1.0,
and Linux version 4.0.0-ADI-1.1.0.

This repository can be used as `linkportdma` package within your `br2-external`
tree.

## License

The project license is specified in COPYING.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

