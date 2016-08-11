#include <ti/drivers/rf/RF.h>
/* RF hwi and swi priority */
const RFCC26XX_HWAttrs RFCC26XX_hwAttrs = { 
    .hwiCpe0Priority = ~0, 
    .hwiHwPriority   = ~0, 
    .swiCpe0Priority =  0,  
    .swiHwPriority   =  0,  
};
