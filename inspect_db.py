import sqlite3
import os

for db_name, rel_path in [('Vci.db', r'Vci\Vci.db'), ('XRef.db', r'XRef\XRef.db')]:
    p = os.path.join(r'c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3 (1)', rel_path)
    try:
        conn = sqlite3.connect(p)
        cur = conn.cursor()
        cur.execute("SELECT name FROM sqlite_master WHERE type='table';")
        tables = cur.fetchall()
        print(f"=== {db_name} Tables ===", tables)
        for t in tables[:5]:
            cur.execute(f"SELECT * FROM {t[0]} LIMIT 3;")
            rows = cur.fetchall()
            print(f"  Table {t[0]} ({len(rows)} rows sample):", rows[:2])
        conn.close()
    except Exception as e:
        print(f"Error {db_name}: {e}")
