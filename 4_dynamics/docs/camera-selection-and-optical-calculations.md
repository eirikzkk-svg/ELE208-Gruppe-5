# Camera selection and optical calculations

Camera and lens selection, nominal coverage and acquisition assumptions supporting the report's Sensor System and Transforms section (3.2). The F labels retain the original appendix equation and table identifiers.

## Camera and lens

We selected a Basler ace 2 a2A1920-160ucBAS colour camera with a 1920 × 1200 global-shutter sensor and 3.45 µm pixels [Basler camera], paired with a FUJINON HF6XA-5M nominal 6 mm C-mount lens [Fujifilm lens]. A calibrated 2D view can locate separated bars and their in-plane orientation when the bar-top height is known.

For the optical calculation, we assume a nominal 1.00 m working distance normal to the bar-top plane. The report's Section 3.2 describes the camera placement and coordinate transformation.

## Optical calculation

Pixel count and pitch give a 6.624 × 4.140 mm sensor area. With nominal focal length $f=6$ mm, the pinhole approximation gives the coverage in Table F.1 [Corke textbook]. Here W and H correspond to the image width and height, D is the working distance and sₓ and sᵧ are the sensor width and height.

$$
W \approx \frac{D s_x}{f},\qquad H \approx \frac{D s_y}{f}
\tag{F.1}
$$

**Table F.1. Calculated image-axis coverage at the nominal 1.00 m working distance.**

| Quantity | Calculated value |
|---|---|
| Field of view (image width × height) | 1.104 × 0.690 m |
| Object-plane sampling | 0.575 mm/pixel |

## Acquisition and calibration

The proposed 60 frames/s and 1 ms exposure imply 5 mm of belt travel between frames and 0.30 mm during exposure at 0.30 m/s. These settings require suitable lighting.

Calibration must account for lens distortion and bar height and be checked against known positions across the belt. Image timestamps and conveyor speed are used to predict the bar's pickup position.

## Sources

- **Basler camera:** Basler AG, [*a2A1920-160ucBAS*](https://www.baslerweb.com/en/shop/a2a1920-160ucbas/). This is the model page used by the report's existing camera specification citation.
- **Fujifilm lens:** FUJIFILM Corporation, [*HF6XA-5M Specifications*](https://asset.fujifilm.com/www/is/files/2021-09/3bc372836503cfaa2f998cb5f943f488/HF6XA-5M_Specifications.pdf), Ver.2.0, 10 May 2019. The optical calculation uses the stated nominal 6 mm focal length.
- **Corke textbook:** Peter Corke, [*Robotics, Vision and Control: Fundamental Algorithms in MATLAB*, 2nd ed.](https://link.springer.com/book/10.1007/978-3-319-54413-7), Springer, 2017, Image Formation, pp. 319–357. Source for the pinhole method; the numerical coverage values are our calculations.

[Basler camera]: https://www.baslerweb.com/en/shop/a2a1920-160ucbas/
[Fujifilm lens]: https://asset.fujifilm.com/www/is/files/2021-09/3bc372836503cfaa2f998cb5f943f488/HF6XA-5M_Specifications.pdf
[Corke textbook]: https://link.springer.com/book/10.1007/978-3-319-54413-7
