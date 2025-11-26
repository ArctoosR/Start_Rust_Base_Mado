For Install On Windows!

# Install git, if you don't have it
curl -A MS https://webinstall.dev/git | powershell

# Install vcpkg
git clone https://github.com/microsoft/vcpkg
.\vcpkg\bootstrap-vcpkg.bat

rustup update
rustup toolchain install stable
rustup toolchain install 1.59.0
rustup default 1.59.0
Note: see https://rustup.rs for Windows 10.

Create Folder [Blue_Pill]
cd Blue_Pill

rustup override set 1.59.0

cargo install cargo-flash --version 0.12.1


rustup target install thumbv7m-none-eabi


# Install ftdi
vcpkg install libftdi1:x64-windows-static-md libusb:x64-windows-static-md


 Install cargo flash for Rust 1.59.0:
 rustup run 1.59.0 \
	cargo install cargo-flash --version 0.12.1
rustup run 1.59.0 \
	cargo install cargo-embed --version 0.12.0
  
Clone the docs repository
git clone https://cgit.pinealservo.com/BluePill_Rust/resources.git ./resources


Clone the code repository:
git clone https://cgit.pinealservo.com/BluePill_Rust/blue_pill_base ./base/
cd base


probe-rs list
"The following devices were found:
[0]: STLink V2 (VID: 0483, PID: 3748, Serial: nGS8LN, STLink)"


Build the sample code
# A) cargo flash (will build and deploy)
cargo flash --release --chip STM32F103C8








