#!/usr/bin/env python

import ntplib
from datetime import datetime
c = ntplib.NTPClient()
# Provide the respective ntp server ip in below function
response = c.request('uk.pool.ntp.org', version=3)
response.offset 
# UTC timezone used here, for working with different timezones you can use [pytz library][1]
print (response.tx_time)


# import logging
# logging.basicConfig(filename='time_shift.txt',level=logging.DEBUG)

# import ntplib
# import time
# import datetime
# c = ntplib.NTPClient()

# interval_sec = 2
# while True:
#     try:
#         response = c.request('europe.pool.ntp.org', version=3)
#         txt = '%s     %+.7f' % (datetime.datetime.now().isoformat(), response.offset)
#         print txt
#         logging.info(txt)
#     except:
#         pass
#     time.sleep(interval_sec)