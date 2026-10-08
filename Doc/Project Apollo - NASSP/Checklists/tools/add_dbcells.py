#!/usr/bin/env python3
"""Rewrite an xlwt-produced BIFF8 .xls so every worksheet carries INDEX + DBCELL records.

xlwt never writes INDEX/DBCELL. NASSP's BasicExcel loader
(src_aux/BasicExcelVC6.cpp, Worksheet::CellTable::RowBlock::Read) loops until it
sees a DBCELL with no bounds check, so such files crash Saturn5.dll.

This tool keeps every ROW and cell record byte-for-byte, only regrouping them into
Excel-style 32-row blocks, each terminated by a DBCELL, inserts an INDEX record
right after each worksheet BOF (plus a DEFCOLWIDTH record, which the INDEX points to,
if the sheet has none), and fixes the BOUNDSHEET stream offsets. The CFB container is
rewritten with xlwt's own CompoundDoc writer (same layout as the original file).

usage: add_dbcells.py in.xls out.xls
"""
import struct, sys

ROW, DBCELL, INDEX, BOF, EOF, BOUNDSHEET = 0x0208, 0x00D7, 0x020B, 0x0809, 0x000A, 0x0085
DEFCOLWIDTH, COLINFO, DIMENSIONS = 0x0055, 0x007D, 0x0200
CELL = {0x0201, 0x0205, 0x00FD, 0x00BE, 0x00BD, 0x0203, 0x027E, 0x0006, 0x0204, 0x00D6}
CELL_TRAIL = {0x0207, 0x04BC, 0x0221, 0x0236}   # STRING/SHRFMLA/ARRAY/TABLEOP follow a FORMULA
CONTINUE = 0x003C


def read_workbook_stream(path):
    import olefile
    with olefile.OleFileIO(path) as ole:
        name = 'Workbook' if ole.exists('Workbook') else 'Book'
        return ole.openstream(name).read()


def records(data):
    out, pos = [], 0
    while pos + 4 <= len(data):
        code, ln = struct.unpack_from('<HH', data, pos)
        if code == 0 and ln == 0 and not any(data[pos:]):
            break                       # zero padding after the last EOF
        out.append((code, data[pos + 4:pos + 4 + ln]))
        pos += 4 + ln
    return out


def rec(code, body):
    return struct.pack('<HH', code, len(body)) + body


def split_substreams(recs):
    subs, cur = [], None
    for code, body in recs:
        if code == BOF:
            cur = []
            subs.append(cur)
        cur.append((code, body))
    return subs


def cell_row(code, body):
    return struct.unpack_from('<H', body, 0)[0]


def rebuild_sheet(sub, sheet_start):
    """Return bytes of a worksheet substream with INDEX/DBCELL, given its absolute start offset."""
    # locate the cell table: first ROW/cell record .. last ROW/cell(+trailers) record
    idx = [i for i, (c, _) in enumerate(sub) if c in CELL or c == ROW]
    if not idx:
        first = last = None
    else:
        first, last = idx[0], idx[-1]
        while last + 1 < len(sub) and sub[last + 1][0] in CELL_TRAIL | {CONTINUE}:
            last += 1
    if any(c in (INDEX, DBCELL) for c, _ in sub):
        raise SystemExit('sheet already has INDEX/DBCELL; refusing to touch it')

    head = sub[:first] if first is not None else sub[:-1]
    table = sub[first:last + 1] if first is not None else []
    tail = sub[last + 1:] if first is not None else sub[-1:]

    # group: rows[r] = ROW body ; cells[r] = list of raw cell records (with trailers) in original order
    rows, cells, cur_row = {}, {}, None
    for code, body in table:
        if code == ROW:
            r = struct.unpack_from('<H', body, 0)[0]
            if r in rows:
                raise SystemExit('duplicate ROW %d' % r)
            rows[r] = body
        elif code in CELL:
            cur_row = cell_row(code, body)
            cells.setdefault(cur_row, []).append(rec(code, body))
        elif code in CELL_TRAIL or code == CONTINUE:
            cells[cur_row][-1] += rec(code, body)
        else:
            raise SystemExit('unexpected record 0x%04x inside cell table' % code)
    for r in cells:
        if r not in rows:
            raise SystemExit('cells for row %d without ROW record' % r)

    # head: BOF, then INDEX, then the rest; add DEFCOLWIDTH before first COLINFO/DIMENSIONS if absent
    head = list(head)
    if not any(c == DEFCOLWIDTH for c, _ in head):
        at = next(i for i, (c, _) in enumerate(head) if c in (COLINFO, DIMENSIONS))
        head.insert(at, (DEFCOLWIDTH, struct.pack('<H', 8)))

    blocks = []
    for r in sorted(rows):
        if not blocks or r // 32 != blocks[-1][0]:
            blocks.append((r // 32, []))
        blocks[-1][1].append(r)

    index_len = 4 + 16 + 4 * len(blocks)
    pos = sheet_start
    head_bytes = rec(*head[0])
    pos += len(head_bytes) + index_len
    rest_head = b''
    defcol_pos = None
    for code, body in head[1:]:
        if code == DEFCOLWIDTH:
            defcol_pos = pos + len(rest_head)
        rest_head += rec(code, body)
    pos += len(rest_head)

    table_bytes, dbcell_pos = b'', []
    for _, rlist in blocks:
        block_start = pos + len(table_bytes)
        row_bytes = b''.join(rec(ROW, rows[r]) for r in rlist)
        offs, cell_bytes = [], b''
        ref = len(row_bytes) - 20          # start of 2nd ROW record -> first cell of 1st row
        for r in rlist:
            cb = b''.join(cells.get(r, []))
            if cb:
                offs.append(ref + 0)
                ref = len(cb)
            else:
                offs.append(0)
                ref += 0
            cell_bytes += cb
        dbpos = block_start + len(row_bytes) + len(cell_bytes)
        db_body = struct.pack('<I', dbpos - block_start) + b''.join(struct.pack('<H', o) for o in offs)
        table_bytes += row_bytes + cell_bytes + rec(DBCELL, db_body)
        dbcell_pos.append(dbpos)

    rw_mic = min(rows) if rows else 0
    rw_mac = max(rows) + 1 if rows else 0
    index_body = struct.pack('<IIII', 0, rw_mic, rw_mac, defcol_pos or 0) + b''.join(struct.pack('<I', p) for p in dbcell_pos)
    out = head_bytes + rec(INDEX, index_body) + rest_head + table_bytes + b''.join(rec(c, b) for c, b in tail)
    assert len(rec(INDEX, index_body)) == index_len
    return out


def main(src, dst):
    data = read_workbook_stream(src)
    subs = split_substreams(records(data))
    glob = subs[0]
    assert struct.unpack_from('<H', glob[0][1], 2)[0] == 0x0005, 'first substream is not workbook globals'
    sheets = subs[1:]
    bs_idx = [i for i, (c, _) in enumerate(glob) if c == BOUNDSHEET]
    assert len(bs_idx) == len(sheets), (len(bs_idx), len(sheets))
    glob_len = sum(4 + len(b) for _, b in glob)

    pos, sheet_bytes = glob_len, []
    for i, sub in enumerate(sheets):
        btype = struct.unpack_from('<H', sub[0][1], 2)[0]
        assert btype == 0x0010, 'substream %d is not a worksheet' % i
        body = glob[bs_idx[i]][1]
        glob[bs_idx[i]] = (BOUNDSHEET, struct.pack('<I', pos) + body[4:])
        sb = rebuild_sheet(sub, pos)
        sheet_bytes.append(sb)
        pos += len(sb)
    stream = b''.join(rec(c, b) for c, b in glob) + b''.join(sheet_bytes)

    from xlwt.CompoundDoc import XlsDoc
    XlsDoc().save(dst, stream)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
