# Apollo checklist BIFF tools

`add_dbcells.py` rewrites an xlwt BIFF8 workbook so every worksheet has one INDEX record and 32-row blocks ended by DBCELL. NASSP loads these files with `Worksheet::CellTable::RowBlock::Read` in `src_aux/BasicExcelVC6.cpp`, which walks records until it sees a DBCELL. xlwt never writes INDEX or DBCELL, so a raw xlwt file runs off the buffer and crashes Orbiter in `memcpy` while a scenario loads.

There is no checklist generator script in this repository. Any script that writes `Apollo 14 Checklists.xls` (or another NASSP checklist) with xlwt must run `add_dbcells.py` as the last step and replace the workbook with that output:

```
python3 add_dbcells.py "in.xls" "Apollo 14 Checklists.xls"
python3 verify_xls.py "in.xls" "Apollo 14 Checklists.xls"
python3 dbcheck.py "Apollo 14 Checklists.xls"
```

`verify_xls.py` checks INDEX/DBCELL layout and that every cell (type, value, xf) matches the input. `dbcheck.py` prints per-sheet DBCELL and INDEX counts; it needs `xlsdiag.py` in this directory.

A LibreOffice "MS Excel 97" round-trip still writes zero DBCELL records. Gnumeric `ssconvert` stores non-ASCII strings as UTF-16, which BasicExcel drops. Neither is a substitute for `add_dbcells.py`.
