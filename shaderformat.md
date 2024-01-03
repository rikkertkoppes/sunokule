This is the contents of a shader packer

| offset      | size | description       |
| ----------- | ---- | ----------------- |
| 0           | 1    | version (1)       |
| 1           | 1    | shader id         |
| 2           | 2    | <memstart>        |
| 4           | 2    | <memsize>         |
| 6           | 2    | <progstart>       |
| 8           | 2    | <progsize>        |
| ...         | ...  | additional data   |
| <memstart>  | 4    | value             |
| +4          | 4    | value             |
| +8          | 4    | value             |
| ...         | ...  | ...               |
| <progstart> | 1    | instruction       |
| +1          | n    | param pointers    |
| +1+n        | 1    | result pointer    |
| ...         | ...  | more instructions |
