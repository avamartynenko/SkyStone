Set-Location -Path 'C:\Users\KZ75L7\code\Android\Sdk\platform-tools'
$addr_string = (.\adb shell ifconfig wlan0) | Select-String 'inet addr' | %{$_.line.trim()}
$phone_addr = $addr_string.Split(' ')[1].split(':')[1]
.\adb tcpip 5555
.\adb connect $phone_addr
