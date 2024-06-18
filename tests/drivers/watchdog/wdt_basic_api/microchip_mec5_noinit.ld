
SECTION_PROLOGUE (.mec5_bootrom_noinit, 0x126400 (NOLOAD), )
{
    __mec5_bootrom_noinit_start = .;
    KEEP(*(SORT_BY_NAME(".mec5_bootrom_noinit*")))
    __mec5_bootrom_noinit_end = .;
} GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)
