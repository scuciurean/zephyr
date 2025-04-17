#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_REGULATOR_HUDSON_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_REGULATOR_HUDSON_H_

/**
 * @defgroup regulator_hudson Hudson PMIC Devicetree helpers.
 * @ingroup regulator_interface
 * @{
 */
/**
    * @name Hudson PMIC regulator modes
    * @{
    */
/** Buck mode */
#define HUDSON_DCDC_MODE_PFM                0x08
#define HUDSON_DCDC_MODE_FPWM               0x09
#define HUDSON_DCDC_MODE_IDLE               0x04
#define HUDSON_DCDC_MODE_HIGH_IMPEDANCE     0x0A
#define HUDSON_DCDC_MODE_SHUTDOWN           0x00

/** @} */ /* end of Hudson PMIC regulator modes */
/** @} */ /* end of regulator_hudson */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_REGULATOR_HUDSON_H_ */
