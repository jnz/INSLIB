// AUTO-GENERATED TEST VECTORS - DO NOT EDIT
// Generated using pygeomag (Exact Spherical Harmonics model)
// Generator (c) Jan Zwiener (jan@zwiener.org); WMM data: NOAA/NCEI via pygeomag.
// NOTE: Compare these expected values against the C interpolation
// using an epsilon (e.g., +/- 0.6 degrees tolerance).
#pragma once

typedef struct {
    float lat;
    float lon;
    float year;
    float expected_decl;
} wmm_test_vector_t;

const wmm_test_vector_t WMM_TEST_VECTORS[8] = {
    {   48.137f,   11.575f, 2027.500f,    4.447f },
    {   40.712f,  -74.006f, 2025.000f,  -12.534f },
    {   35.689f,  139.692f, 2029.900f,   -8.058f },
    {   89.900f,    0.000f, 2026.000f,   15.597f },
    {  -89.900f,  179.900f, 2026.000f,  148.302f },
    {    0.000f,  180.000f, 2028.000f,   10.012f },
    {    0.000f, -180.000f, 2028.000f,   10.012f },
    {    0.000f,  370.000f, 2027.000f,   -0.897f },
};
const int WMM_TEST_VECTOR_COUNT = 8;
