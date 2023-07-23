| offset       | size | description       |
| ------------ | ---- | ----------------- |
| 0            | 1    | version (1)       |
| 1            | 1    | shader id         |
| 2            | 2    | <memoffset>       |
| 4            | 2    | <memsize>         |
| 6            | 2    | <progoffset>      |
| 8            | 2    | <progsize>        |
| ...          | ...  | additional data   |
| <memoffset>  | 4    | value             |
| +2           | 4    | value             |
| +6           | 4    | value             |
| ...          | ...  | ...               |
| <progoffset> | 1    | instruction       |
| +1           | n    | param pointers    |
| +1+n         | 1    | result pointer    |
| ...          | ...  | more instructions |
