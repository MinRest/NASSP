"""Workbook-stream reader used by dbcheck.py.

dbcheck.py execs this file up to the marker line below and calls
cfb_stream(path), which must return a 4-tuple whose first element is the
BIFF workbook stream.
"""
import olefile


def cfb_stream(path):
    with olefile.OleFileIO(path) as ole:
        name = "Workbook" if ole.exists("Workbook") else "Book"
        data = ole.openstream(name).read()
    return data, None, None, None


CODES = {}
