from pathlib import Path
import hashlib,json
p=Path('@DIAGNOSTICS@')
before=(p/'preflash-full.bin').read_bytes()
after=(p/'boot-journal-after.bin').read_bytes()
result=dict(backup_bytes=len(before),backup_sha256=hashlib.sha256(before).hexdigest(),boot_journal_bytes=len(after),boot_journal_preserved=len(before)==524288 and len(after)==8192 and before[-8192:]==after,raw_backups_remain_on_jetson=True)
(p/'backup-verification.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
raise SystemExit(0 if result['boot_journal_preserved'] else 2)
