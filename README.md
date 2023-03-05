# Suno Kule firmware

Follow the following steps to set up your hardware and firmware

-   connect led strips to pin 25 and optionally 26 and 27 of an esp32
-   install espressif idf extension in vscode if needed
-   change `mapping.h` to your own layout
-   configure suno kule options:
    -   change pin numbers if needed
    -   adjust led counts if needed
    -   set wifi ssid and password
-   build and flash
-   note the ip address in the serial monitor
-   go to https://suno-kule.vercel.app and connect to your firmware

# generating ssl certificate for https

some self signed certificates are shipped in the build, you can create your own:

```
openssl req -newkey rsa:2048 -nodes -keyout prvtkey.pem -x509 -days 3650 -out cacert.pem -subj "/CN=ESP32 suno kule"
```
