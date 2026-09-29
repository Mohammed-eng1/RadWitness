# Verifier — `radwitness.py`

Builds a tamper-evident mission log and checks it.

- **Chain:** every entry stores the SHA-256 fingerprint of the entry before it.
- **Seal:** at mission end, the last fingerprint and the number of entries are signed with Ed25519.
- **Verify:** anyone with the robot's public key can recompute the chain and check the seal. If something changed, it reports the exact entry and the reason.

Needs Python 3 and the `cryptography` package.

```bash
python3 radwitness.py keygen                    # once: creates keys/robot.key + keys/robot.pub
python3 radwitness.py from-csv mission.csv LOG  # build a log from mission data
python3 radwitness.py seal LOG                  # seal at mission end
python3 radwitness.py verify LOG                # check it
python3 radwitness.py show LOG                  # print the entries
python3 radwitness.py tamper-test LOG --n 1000  # random-edit test
```

**Never share or upload `keys/robot.key`.** Only `robot.pub` is given to the receiver, in advance.

Messages printed by the tool are in Arabic.
