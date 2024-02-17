import os
from datetime import datetime
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

pic_time=datetime.now() .strftime('%Y%m%d%H%M%S')
pic_time = datetime.now()
command = 'raspistill -w 1280 -h 720 -vf -hf -o' + pic_time + '.jpg'
os.system(command)

#Email
smtpUser = 'naveenbot003@gmail.com'
smtpPass = 'gzfqivvfwdxwhmyd'

#Destination
toAdd = 'ENPM809TS19@gmail.com'
fromAdd = smtpUser
subject = 'Block picked at ' + pic_time
msg = MIMEMultipart()
msg['Subject'] = subject
msg['From'] = fromAdd
msg['To'] = toAdd
#Attach image
'''
fp = open(pic_time + '.jpg','rb')
img = MIMEImage(fp.read())
fp.close()
msg.attach(img)
'''
#Send
se = smtplib.SMTP('smtp.gmail.com',587)

se.ehl0()
se.starttls
se.ehlo()
se.login(smtpUser, smtpPass)
se.sendmail(fromAdd, toAdd, msg.as_string())
se.quit()
print("Email delivered")
