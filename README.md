# Suno Kule firmware

Follow the following steps to set up your hardware and firmware

-   connect led strips to pin 25 and optionally 26 and 27 of an esp32
-   use esp idf version 4.4.x
-   install espressif idf extension in vscode if needed
-   change `mapping.h` to your own layout
-   configure suno kule options:
    -   change pin numbers if needed
    -   adjust led counts if needed
-   build and flash
-   note the ip address in the serial monitor
-   connect to the ap that is set up by the esp and visit https://192.168.4.1
-   enter your wifi credentials
-   visit the device on the new address
-   note that you get a warning visiting over https as the ssl certificate is self signed
-   go to https://suno-kule.vercel.app and connect to your firmware. You need to have accepted the earlier warning for it to connect properly

# generating ssl certificate for https

some self signed certificates are shipped in the build, you can create your own:

```
openssl req -newkey rsa:2048 -nodes -keyout prvtkey.pem -x509 -days 3650 -out cacert.pem -subj "/CN=ESP32 suno kule"
```
