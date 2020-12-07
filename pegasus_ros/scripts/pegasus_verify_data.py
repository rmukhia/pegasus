#!/usr/bin/env python
import hashlib
from hmac import compare_digest

HASH_SIZE = 16


class VerifyError(Exception):
    pass


def verify_msg(msg):
    size = len(msg)
    if size <= HASH_SIZE:
        raise VerifyError('message too short')
    received_digest = msg[0:16]
    data = msg[16:]
    m = hashlib.md5()
    m.update(data)
    calculated_digest = m.digest()
    if not compare_digest(calculated_digest, received_digest):
        raise VerifyError('hash does not match %s , %s' % (received_digest, calculated_digest))
    return data


def pack_msg(msg):
    size = len(msg)
    m = hashlib.md5()
    m.update(msg)
    return m.digest() + msg


if __name__ == '__main__':
    data = b'verify from corruption'
    msg1 = pack_msg(data)
    result = verify_msg(msg1)
    print (result)
    result = verify_msg(msg1 + '.')
    print (result)
