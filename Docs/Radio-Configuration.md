# Connecting to the Radio
Connecting to the radio can be done in multiple ways:

- Ethernet
- Wifi
- Configuring your IpV4

Ethernet: Get an ethernet wire and plug it into the ethernet port of your laptop and the ethernet port of the radio (Aux2 port).

Wifi: Click on the Wifi Logo in the bottom right corner of the screen and navigate to the Wifi network dropdown page (Wifi Symbol with Arrow), once there click on your radio's Wifi network (Ex. FRC-839-Rosie).

Configuring your IpV4: Navigate to the settings on your laptop to "Network and Internet." Then while NOT connected to any Wifi or Ethernet navigate to the "Wi-Fi" button and click on "Hardware Properties." Once there scroll down to "More Adapter Options" and click on edit. Then left click on "Internet Protocol Version 4 (TCP/IPv4)" and select properties. Switch both the "Obtain IP Address Automatically" and "Obtain DNS server address automatically" to the manual option. Configure IP address to "10.8.39.1", Subnet mask to "255.255.255.0", Leave the Default Gateway blank, and change the Preferred DNS server to "10.8.39.1."

# Configuring your Radio
When a radio is purchased, it needs to be configured (Some radio problems are also due to a need of configuration)

Connect to your radio through Ethernet. Once connected navigate in your browser to [Vivid Hosting Documentation](https://frc-radio.vivid-hosting.net/getting-started/usage/programming-your-radio) and follow the instructions listed on the website.

After you click the Configure button on the configuring page, it has to apply the configuration to the Radio which will cause the radio itself to reboot a few times; you will have to wait about a minute or two.

# Updating the Firmware on Your Radio

Make sure you are connected via Ethernet to the Radio.

Keep the radio up to date with firmware by check the VividHosting website with the latest updates.

Use this link to update the firmware on your radio [Vivid Hosting Update Documentation](https://frc-radio.vivid-hosting.net/miscellaneous/upgrading-firmware). 
*Note: VividHosting may use a different IP address on their documentation, instead of using that address use our address "10.8.39.1".*
*Note: For now we are using the Non-Access-Point firmware, might change in the future*

Releases for the firmware can be found at [VividHosting Firmware Releases](https://frc-radio.vivid-hosting.net/miscellaneous/firmware-releases).

Once Firmware is updated you need to check the status on the Radio. To do this naviagte to your Radio's IP address "10.8.39.1" and replace /configuration with /status. It should look like this "http://10.8.39.1/status".
