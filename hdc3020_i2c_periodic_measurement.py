# -*- coding: utf-8 -*-
"""
Example script reading measurement values from the HDC3020 Sensor via I2c interface.

Copyright 2024 MW technologies

Disclaimer:
This application example is non-binding and does not claim to be complete with regard
to configuration and equipment as well as all eventualities. The application example
is intended to provide assistance with the HDC3020 sensor module design-in and is provided "as is".
You yourself are responsible for the proper operation of the products described.
This application example does not release you from the obligation to handle the product safely
during application, installation, operation and maintenance. By using this application example,
you acknowledge that we cannot be held liable for any damage beyond the liability regulations
described.

We reserve the right to make changes to this application example at any time without notice.
In case of discrepancies between the suggestions in this application example and other MW technologies
publications, such as catalogues, the content of the other documentation takes precedence.
We assume no liability for the information contained in this document.
"""



import time
from hdc3020_i2c_library import HDC3020


# Definition
CSV_DELIMETER = ","


HDC_3020 = HDC3020(0x44)

# read device identification
try:
    print("identification: " + ''.join('{:02x}'.format(x) for x in HDC_3020.read_identification()))
    HDC_3020.change_periodic_measurement_time(5000)
    print("periodic measurement time interval is: "+str(HDC_3020.read_periodic_measurement_time()) +
          " s")
except Warning as exception:
    print("Exception: " + str(exception))

HDC_3020.start_periodic_measurement(1,0)
# print csv header
print("temperature", CSV_DELIMETER,
      "relative humidity", CSV_DELIMETER,
      "dewpoint")

for i in range(30):

    try:
        temperature,humidity = HDC_3020.get_periodic_measurement_temp_hum()
        dewpoint = HDC_3020.get_dewpoint(temperature,humidity)

        print('%0.2f °C' % temperature, CSV_DELIMETER,
              '%0.2f %%RH' % humidity, CSV_DELIMETER,
              '%0.2f °C' % dewpoint)

    except Warning as exception:
        print("Exception: " + str(exception))

    finally:
        time.sleep(1)
HDC_3020.end_periodic_measurement()
