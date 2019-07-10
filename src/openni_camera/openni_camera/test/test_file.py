#! /usr/bin/env python

import shutil
import tempfile

tmp = tempfile.TemporaryFile(mode='r+')
tmp.write("This is a test\n")
f = open("my_file.txt", 'w')
tmp.seek(0)
shutil.copyfileobj(tmp, f)
