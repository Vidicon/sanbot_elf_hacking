def ihex_to_byte_list_with_print(ihex_lines):
    for line in ihex_lines.splitlines():
        if line.startswith(":"):
            record_length = int(line[1:3], 16)
            data = line[9 : 9 + record_length * 2]
            byte_list = [f"0x{data[i:i+2]}" for i in range(0, len(data), 2)]
            print(byte_list)


ihex_dump = """
:10FB7C000000000000000000000000000000000079
:10FB8C000000000000000000000000000000000069
:10FB9C000000020206060E0E1E1C1C3C7C78F8F8B7
:10FBAC00F8F8F0F0F0F0E0E0E0C0C0C080800000B9
:10FBBC000000000000000000000000000000000039
:10FBCC000000000000000000000000000000000029
:10FBDC000000000000000000000000000000000019
:10FBEC000000000000000000000000000000000009
:10FBFC0000000000000000000000000000000000F9
:10FC0C0000000000000000000000000000000000E8
:10FC1C0000000000000000000080C0E0F0F87C3F15
:10FC2C001F0F07070707070F0F1F1F3F3F7F7FFFA0
:10FC3C00FEFEFCFCF8F0F0E0C0C08000000000000C
:10FC4C0000000000000000000000000000000000A8
:10FC5C000000000000000000000000000000000098
:10FC6C000000000000000000000000000000000088
:10FC7C000000000000000000000000000000000078
:10FC8C000000000000000000000000000000000068
:10FC9C000000000000E0F0FCFE7F1F0F03000000DE
:10FCAC00000000000000000000000038F0E08000C0
:10FCBC0000C18383070F1F3F7FFFFFFFFFFEFCF890
:10FCCC00F0E0C08000000000000000000000000018
:10FCDC000000000000000000000000000000000018
:10FCEC000000000000000000000000000000000008
:10FCFC0000000000000000000000000000000000F8
:10FD0C0000000000000000000000000000000000E7
:10FD1C000000C0F8FFFFFF1F030000000000000000
:10FD2C0000000000000000C0E0F03800000FFFFFF2
:10FD3C00FCF0C0038FFEFCF0C0000103070F1F3F57
:10FD4C007FFFFFFFFFFEFCF8F0E0C080000000002A
:10FD5C000000000000000000000000000000000097
:10FD6C000000000000000000000000000000000087
:10FD7C000000000000000000000000000000000077
:10FD8C000000000000000000000000000000000067
:10FD9C0000007FFFFFFFFFE00000000000000000FC
:10FDAC0000000000E0FCFFFFFFFFFCF0F0F8FFFF9D
:10FDBC00FFFFFFFFFFFFFFFFFFFFFCE00000000065
:10FDCC0000000103070F1FFFFFFFFFFFFFFEFCF802
:10FDDC00F0E0C0C0808080808080C0C0E0E0F03067
:10FDEC000000000000000000000000000000000007
:10FDFC0000000000000000000000000000000000F7
:10FE0C0000000000000000000000000000000000E6
:10FE1C00000000030F3FFFFFFFF8E0C000000000F0
:10FE2C00000000001F7FFFFFFFFFFFFFFFFFFFFF32
:10FE3C00FFFFFFFFFFFFFFFFFFFFFF1F00000000A2
:10FE4C0000000000C0F0FCFFFF3F0F03070F1F1F57
:10FE5C003F3F7F7F7F7F7F3F3F3F1F0F07010000AA
:10FE6C000000000000000000000000000000000086
:10FE7C000000000000000000000000000000000076
:10FE8C000000000000000000000000000000000066
:10FE9C000000000000000003070F1F3F7FFEFCF86E
:10FEAC00F0E0E0C08080010307070F0F1F1F1F1F2A
:10FEBC001F1F1F1F0F0F07070301008080C0C0E02A
:10FECC00F0F87C3F1F0F070300000000000000004B
:10FEDC000000000000000000000000000000000016
:10FEEC000000000000000000000000000000000006
:10FEFC0000000000000000000000000000000000F6
:10FF0C0000000000000000000000000000000000E5
:10FF1C0000000000000000000000000000000001D4
:10FF2C00030307070F0F1F1F1F1E3E3E3E7C7C7CEA
:10FF3C007C7C7C3C3C3E1E1E1F0F0F0F07070303EF
:10FF4C0001000000000000000000000000000000A4
:10FF5C000000000000000000000000000000000095
:10FF6C000000000000000000000000000000000085
"""

bytes_list = ihex_to_byte_list_with_print(ihex_dump)
print(bytes_list)
