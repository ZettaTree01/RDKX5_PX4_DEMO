# -*- coding: utf-8 -*-
"""把本地 02 例程同步到机载电脑 /app/zettatree_demo。

源码或 README 改完后运行本脚本。不上传临时文件与 __pycache__。
"""
import os
import stat

import paramiko

HOST = "192.168.101.168"
USER = "sunrise"
PASSWORD = "sunrise"
LOCAL_ROOT = r"D:\gs-workspace\02"
REMOTE_ROOT = "/app/zettatree_demo"
SKIP_PREFIX = ("_tmp_", "_update_tutorial.py", "_sync_to_x5.py")
SKIP_DIR = {"__pycache__"}
SKIP_NAMES = {".gitignore"}
EXEC_EXT = {".py", ".sh"}


def should_skip(rel):
    name = os.path.basename(rel)
    if name in SKIP_PREFIX or name in SKIP_NAMES or name.startswith("_tmp_"):
        return True
    parts = rel.replace("\\", "/").split("/")
    return any(p in SKIP_DIR for p in parts)


def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASSWORD, timeout=20)
    sftp = c.open_sftp()

    def mkdir_p(path):
        try:
            sftp.stat(path)
            return
        except OSError:
            pass
        parent = os.path.dirname(path)
        if parent and parent != path:
            mkdir_p(parent)
        sftp.mkdir(path)

    uploaded = 0
    for dirpath, dirnames, filenames in os.walk(LOCAL_ROOT):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIR]
        for name in filenames:
            local = os.path.join(dirpath, name)
            rel = os.path.relpath(local, LOCAL_ROOT)
            if should_skip(rel):
                continue
            remote = REMOTE_ROOT + "/" + rel.replace("\\", "/")
            mkdir_p(os.path.dirname(remote).replace("\\", "/"))
            sftp.put(local, remote)
            if os.path.splitext(name)[1] in EXEC_EXT:
                sftp.chmod(remote, stat.S_IRWXU | stat.S_IRGRP | stat.S_IXGRP
                           | stat.S_IROTH | stat.S_IXOTH)
            uploaded += 1
            print("put", rel)

    sftp.close()
    c.close()
    print("uploaded", uploaded, "files ->", REMOTE_ROOT)


if __name__ == "__main__":
    main()
