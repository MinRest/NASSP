#!/usr/bin/env python3
"""verify_xls.py OLD.xls NEW.xls  -- DBCELL/INDEX structure in NEW + cell-identical check vs OLD (xlrd)."""
import struct, sys, olefile, xlrd

def stream(p):
    with olefile.OleFileIO(p) as o:
        return o.openstream('Workbook').read()

def walk(data):
    pos, recs = 0, []
    while pos + 4 <= len(data):
        c, l = struct.unpack_from('<HH', data, pos)
        if c == 0 and l == 0 and not any(data[pos:]): break
        recs.append((pos, c, data[pos+4:pos+4+l])); pos += 4 + l
    return recs

def structure(p):
    data = stream(p); recs = walk(data); at = {pos: c for pos, c, _ in recs}
    bounds = [(struct.unpack_from('<I', b, 0)[0]) for _, c, b in recs if c == 0x85]
    sheets, cur = [], None
    for pos, c, b in recs:
        if c == 0x809:
            cur = {'pos': pos, 'type': struct.unpack_from('<H', b, 2)[0], 'DBCELL': 0, 'INDEX': 0, 'idx_ok': True, 'db_ok': True, 'ROW': 0}
            sheets.append(cur)
        elif c == 0xD7:
            cur['DBCELL'] += 1
            if at.get(pos - struct.unpack_from('<I', b, 0)[0]) != 0x208: cur['db_ok'] = False
        elif c == 0x208: cur['ROW'] += 1
        elif c == 0x20B:
            cur['INDEX'] += 1
            n = (len(b) - 16) // 4
            for i in range(n):
                if at.get(struct.unpack_from('<I', b, 16 + 4*i)[0]) != 0xD7: cur['idx_ok'] = False
            dc = struct.unpack_from('<I', b, 12)[0]
            if dc and at.get(dc) != 0x55: cur['idx_ok'] = False
    ws = [s for s in sheets if s['type'] == 0x10]
    bof_ok = all(at.get(o) == 0x809 for o in bounds) and len(bounds) == len(ws)
    return ws, bof_ok

def cells(p):
    bk = xlrd.open_workbook(p, formatting_info=True)
    out = []
    for sh in bk.sheets():
        out.append((sh.name, sh.nrows, sh.ncols,
                    [[(sh.cell_type(r, c), sh.cell_value(r, c), sh.cell_xf_index(r, c)) for c in range(sh.ncols)] for r in range(sh.nrows)]))
    return bk.nsheets, out

old, new = sys.argv[1], sys.argv[2]
ws, bof_ok = structure(new)
for i, s in enumerate(ws):
    print('sheet %2d  ROW=%4d  DBCELL=%3d  INDEX=%d  index->dbcell ok=%s  dbcell->row ok=%s' % (i, s['ROW'], s['DBCELL'], s['INDEX'], s['idx_ok'], s['db_ok']))
print('worksheets:', len(ws), '| min DBCELL per sheet:', min(s['DBCELL'] for s in ws), '| total DBCELL:', sum(s['DBCELL'] for s in ws),
      '| every sheet INDEX==1:', all(s['INDEX'] == 1 for s in ws), '| BOUNDSHEET offsets -> BOF:', bof_ok)
no, co = cells(old); nn, cn = cells(new)
names_same = [x[0] for x in co] == [x[0] for x in cn]
print('xlrd sheets old/new:', no, nn, '| names identical:', names_same)
diff = 0; ncell = 0
for a, b in zip(co, cn):
    if a[1:3] != b[1:3]: print('DIM DIFF', a[0], a[1:3], b[1:3]); diff += 1
    for r, (ra, rb) in enumerate(zip(a[3], b[3])):
        for c, (x, y) in enumerate(zip(ra, rb)):
            ncell += 1
            if x != y:
                diff += 1
                if diff < 20: print('CELL DIFF', a[0], r, c, x, y)
print('cells compared (type,value,xf):', ncell, '| differences:', diff)
ok = names_same and no == nn and diff == 0 and bof_ok and all(s['DBCELL'] > 0 and s['INDEX'] == 1 and s['idx_ok'] and s['db_ok'] for s in ws)
print('RESULT:', 'PASS' if ok else 'FAIL')
sys.exit(0 if ok else 1)
