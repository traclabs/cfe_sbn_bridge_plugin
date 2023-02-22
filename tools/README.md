# Wireshark Tooling

## Plugin
This dissector parses SBN message trafic in wireshark.

To install the plugin copy sbn_plugin.lua to your Wireshark user LUA plugins directory, typically ~/.local/lib/wireshark/plugins/sbn_protocol.lua  In Wireshark, you can verify directories by going to Help->About->Directories.

This plugin is configured to decode SBN traffic on UDP ports 2234,2235, and 2236.  If the SBN instance(s) are updated to operate on different ports, update the last few lines of the lua file accordingly.

## TShark Wrapper

The following command provides a convenient command-line view of traffic with tshark
- `tshark -ni lo -Y "sbn && not icmp" -T fields -e frame.number -e udp.srcport -e udp.dstport -e sbn.message_type -e sbn.message_size -e sbn.processor_id -e sbn.spacecraft_id`
- `tshark -ni lo -Y "sbn && not icmp" -V`

A wrapper script is provided in the 'tshark' subdirectoy to provide a parsed view of this data in text or terminal UI (tui) views.

To run:
- `brash_tshark`
- `brash_tshark --tui`
- `brash_tshark --help`

A self-contained binary is available on the bitbucket Downloads page. Alternatively see below to install from source (recommended)

### Setup (source)
- Install NodeJS (https://nodejs.org/en/)
- cd tshark
- npm install      # Install dependencies
- npm install -g . # Install globally for your account


Note: To build a self-contained executable install caxa (`npm install -g caxa`) and run `caxa --input "." --output "brash_tshark" -- "{{caxa}}/node_modules/.bin/node" "{{caxa}}/brash_tsark.js"`
