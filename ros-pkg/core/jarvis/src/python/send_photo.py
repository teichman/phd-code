#!/usr/bin/python

import smtplib, time
import os, sys, string
import argparse
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from email.mime.multipart import MIMEMultipart

# -- Parse args.
if len(sys.argv) == 1:
    print "Usage: send_photo.py PNG TO SUBJ... --body BODY..."
    exit(0)

pngfile = sys.argv[1]
fr = 'jarvis.courier@gmail.com'
to = sys.argv[2]
subj = ""
body = ""

args = sys.argv[3:]
if "--body" not in args:
    subj = " ".join(args)
else:
    subj = " ".join(args[:args.index("--body")])
    body = " ".join(args[args.index("--body")+1:])

print "To:\t" + to
print "Subj:\t" + subj
print "Body:\t" + body
print

# -- Populate the email.
msg = MIMEMultipart()
msg['Subject'] = subj
msg['From'] = fr
msg['To'] = to
msg.preamble = 'preamble'

# Assume we know that the image files are all in PNG format
# Open the files in binary mode.  Let the MIMEImage class automatically
# guess the specific image type.
fp = open(pngfile, 'rb')
img = MIMEImage(fp.read())
fp.close()
msg.attach(img)

emails = string.split(sys.argv[2], ', ')
print 'Sending to: '
for em in emails:
    print em

# Send the message via our own SMTP server, but don't include the
# envelope header.

campus = False
server = []
if campus:
    # This works on campus but not off.
    server = smtplib.SMTP('smtp-unencrypted.stanford.edu')
else:
    # hard coded gmail fallback.
    username = 'jarvis.courier@gmail.com'
    password = 'robotsareawesome'
    server = smtplib.SMTP('smtp.gmail.com:587')
    server.starttls()
    server.login(username,password)

fail = server.sendmail(sys.argv[1], emails, msg.as_string())
if fail:
    print fail
else:
    print ' *** Email sent.'


server.quit()

