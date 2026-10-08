# stdlib-only, read-only: per-worksheet DBCELL/INDEX/ROW counts for one .xls (uses xlsdiag.cfb_stream)
import sys, os, struct
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'xlsdiag.py'), encoding='utf-8-sig').read().split('CODES =')[0])
data, _, _, _ = cfb_stream(sys.argv[1])
pos = 0; sheets = []; cur = None
while pos + 4 <= len(data):
    c, l = struct.unpack_from('<HH', data, pos)
    if c == 0 and l == 0 and not any(data[pos:]): break
    if c == 0x809:
        cur = [struct.unpack_from('<H', data, pos + 6)[0], 0, 0, 0]; sheets.append(cur)
    elif c == 0xD7: cur[1] += 1
    elif c == 0x20B: cur[2] += 1
    elif c == 0x208: cur[3] += 1
    pos += 4 + l
ws = [s for s in sheets if s[0] == 0x10]
print('worksheets=%d  min_DBCELL=%d  total_DBCELL=%d  sheets_with_INDEX=%d  sheets_with_zero_DBCELL=%d' % (
    len(ws), min(s[1] for s in ws), sum(s[1] for s in ws), sum(1 for s in ws if s[2] == 1), sum(1 for s in ws if s[1] == 0)))
print('PASS' if ws and all(s[1] > 0 and s[2] == 1 for s in ws) else 'FAIL')
