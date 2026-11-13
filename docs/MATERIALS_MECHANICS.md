# Materials mechanics for lithocam

Sourced values for granite, marble, limestone, sandstone, a hardwood and a softwood, plus cutting energy, wire data and robot stiffness. Generated from `python/src/lithocam/materials_mechanics.py`; edit that file, not this one.

Read column: "page read" means the page text was fetched and read. "snippet" means only a search excerpt was seen, so check the source before relying on it. Many publisher pages (Springer, MDPI, PMC) refused automated fetches; no login or paywall was bypassed. An empty low and high means nothing was found. Ranges come from different samples and test methods and are not comparable across rows.

## Item 1. Elastic constants, density and strength

| material | quantity | low | high | unit | source (link) | source type | read | note |
|---|---|---|---|---|---|---|---|---|
| sandstone | youngs_modulus | 5 | 50 | GPa | https://tu-freiberg.de/sites/default/files/2023-11/56%20Useful%20Numbers%20for%20Geotechnical%20Engineers%206.pdf | handbook table | snippet | search summary of a geotechnical table, origin not confirmed.  |
| limestone | youngs_modulus | 15 | 70 | GPa | https://tu-freiberg.de/sites/default/files/2023-11/56%20Useful%20Numbers%20for%20Geotechnical%20Engineers%206.pdf | handbook table | snippet | search summary of a geotechnical table, origin not confirmed.  |
| marble | youngs_modulus | 30 | 70 | GPa | https://tu-freiberg.de/sites/default/files/2023-11/56%20Useful%20Numbers%20for%20Geotechnical%20Engineers%206.pdf | handbook table | snippet | search summary of a geotechnical table, origin not confirmed. an older edition of such a table gave 35-60 GPa |
| granite | youngs_modulus | 20 | 70 | GPa | https://tu-freiberg.de/sites/default/files/2023-11/56%20Useful%20Numbers%20for%20Geotechnical%20Engineers%206.pdf | handbook table | snippet | search summary of a geotechnical table, origin not confirmed.  |
| sandstone | poissons_ratio | 0.2 | 0.4 | - | https://tu-freiberg.de/sites/default/files/2023-11/56%20Useful%20Numbers%20for%20Geotechnical%20Engineers%206.pdf | handbook table | snippet | search summary of a geotechnical table, origin not confirmed.  |
| limestone | poissons_ratio | 0.2 | 0.35 | - | https://tu-freiberg.de/sites/default/files/2023-11/56%20Useful%20Numbers%20for%20Geotechnical%20Engineers%206.pdf | handbook table | snippet | search summary of a geotechnical table, origin not confirmed.  |
| granite | poissons_ratio | 0.1 | 0.3 | - | https://tu-freiberg.de/sites/default/files/2023-11/56%20Useful%20Numbers%20for%20Geotechnical%20Engineers%206.pdf | handbook table | snippet | search summary of a geotechnical table, origin not confirmed. typical 0.1-0.2 |
| marble | poissons_ratio | 0.1 | 0.3 | - | https://tu-freiberg.de/sites/default/files/2023-11/56%20Useful%20Numbers%20for%20Geotechnical%20Engineers%206.pdf | handbook table | snippet | search summary of a geotechnical table, origin not confirmed. a separate table gave 0.06-0.22 |
| granite | tensile_strength | 6 | 15 | MPa | https://tu-freiberg.de/sites/default/files/2023-11/56%20Useful%20Numbers%20for%20Geotechnical%20Engineers%206.pdf | handbook table | snippet | search summary of a geotechnical table, origin not confirmed. test method not stated in the summary |
| sandstone | tensile_strength | 1 | 15 | MPa | https://tu-freiberg.de/sites/default/files/2023-11/56%20Useful%20Numbers%20for%20Geotechnical%20Engineers%206.pdf | handbook table | snippet | search summary of a geotechnical table, origin not confirmed. another table gave 4-25 |
| limestone | tensile_strength | 10 | 20 | MPa | https://tu-freiberg.de/sites/default/files/2023-11/56%20Useful%20Numbers%20for%20Geotechnical%20Engineers%206.pdf | handbook table | snippet | search summary of a geotechnical table, origin not confirmed. another table gave 5-25 |
| marble | tensile_strength | 5 | 10 | MPa | https://tu-freiberg.de/sites/default/files/2023-11/56%20Useful%20Numbers%20for%20Geotechnical%20Engineers%206.pdf | handbook table | snippet | search summary of a geotechnical table, origin not confirmed.  |
| marble | density | 2.7 | 2.7 | g/cm3 | https://stoneline.com.tr/en/?p=28479 | vendor | snippet | Carrara datasheet |
| marble | density | 2500 | 2700 | kg/m3 | https://spotmaterial.com/material/marble/slab/ | vendor | snippet | marble in general, materials database |
| marble | poissons_ratio | 0.2 | 0.2 | - | https://spotmaterial.com/material/marble/slab/ | vendor | snippet | marble in general, materials database |
| marble | youngs_modulus | 21.53 | 21.53 | GPa | https://airdrive.eventsair.com/eventsairwesteuprod/production-atlanticmice-public/8f7fdf2218664bc2874fd37f40e7d4d5 | paper | snippet | Carrara, mean tangent modulus from a conference study, standard deviation 8.62 GPa |
| marble | compressive_strength | 75 | 75 | MPa | https://stoneline.com.tr/en/?p=28479 | vendor | snippet | Carrara datasheet, 10930 psi |
| marble | compressive_strength | 100 | 100 | MPa | https://stoneline.com.tr/en/?p=28479 | vendor | snippet | a second Carrara listing gave 1020 kg/cm2, about 100 MPa; origin page not confirmed |
| marble | compressive_strength | 33 | 210 | MPa | https://spotmaterial.com/material/marble/slab/ | vendor | snippet | marble in general |
| marble | tensile_strength | 2.4 | 24.5 | MPa | https://spotmaterial.com/material/marble/slab/ | vendor | snippet | marble in general; test method not stated; no Carrara Brazilian value found |
| limestone | density | 2307 | 2307 | kg/m3 | https://www.polycor.com/stones/indiana-limestone-standard-buff/ | vendor | snippet | Indiana limestone, 144 lb/ft3; the Indiana Limestone Institute minimum for medium density is 135 lb/ft3 |
| limestone | compressive_strength | 27.58 | 27.58 | MPa | https://www.polycor.com/stones/indiana-limestone-standard-buff/ | vendor | snippet | Indiana limestone, 4000 psi, a specified minimum; most products exceed it |
| limestone | flexural_strength | 4.83 | 4.83 | MPa | https://www.polycor.com/stones/indiana-limestone-standard-buff/ | vendor | snippet | Indiana limestone modulus of rupture, 700 psi; flexural, not direct tension |
| sandstone | density | 2147 | 2147 | kg/m3 | https://www.selectstone.com/?p=8110 | vendor | snippet | Berea sandstone, 134 lb/ft3 |
| sandstone | compressive_strength | 79.8 | 79.8 | MPa | https://www.selectstone.com/?p=8110 | vendor | snippet | Berea sandstone, 11576 psi |
| hard rock (unnamed) | density | 2564.76 | 2564.76 | kg/m3 | https://pmc.ncbi.nlm.nih.gov/articles/PMC9573887 | paper | page read | Wang et al. 2022 Sci Rep 12:17346, Table 1; the model itself used 2670 kg/m3 |
| hard rock (unnamed) | compressive_strength | 108.45 | 108.45 | MPa | https://pmc.ncbi.nlm.nih.gov/articles/PMC9573887 | paper | page read | same table; rock type not named |
| hard rock (unnamed) | tensile_strength | 6.67 | 6.67 | MPa | https://pmc.ncbi.nlm.nih.gov/articles/PMC9573887 | paper | page read | same table |
| hard rock (unnamed) | poissons_ratio | 0.209 | 0.209 | - | https://pmc.ncbi.nlm.nih.gov/articles/PMC9573887 | paper | page read | same table |
| granite | density |  |  | kg/m3 | none |  |  | no clean source found in this pass |
| granite | compressive_strength |  |  | MPa | none |  |  | no clean source found in this pass; Kendrick et al. 2023 Adv. Geosci. 62 (G603 granite, UCS and Brazilian) is the lead, not read |
| sandstone | poissons_ratio |  |  | - | none |  |  | only the generic range above was found |
| limestone | tensile_strength |  |  | MPa | none |  |  | no Brazilian value for a named limestone found; only the generic range above |
| hardwood: white oak | specific_gravity_od | 0.68 | 0.68 | - | https://woodbin.com/ref/wood-strength-table/ | handbook | page read | oven-dry specific gravity; the page does not state the moisture basis of the other columns |
| hardwood: northern red oak | specific_gravity_od | 0.63 | 0.63 | - | https://woodbin.com/ref/wood-strength-table/ | handbook | page read |  |
| hardwood: American beech | specific_gravity_od | 0.64 | 0.64 | - | https://woodbin.com/ref/wood-strength-table/ | handbook | page read |  |
| softwood: eastern white pine | specific_gravity_od | 0.35 | 0.35 | - | https://woodbin.com/ref/wood-strength-table/ | handbook | page read |  |
| softwood: loblolly pine | specific_gravity_od | 0.51 | 0.51 | - | https://woodbin.com/ref/wood-strength-table/ | handbook | page read |  |
| softwood: Sitka spruce | specific_gravity_od | 0.4 | 0.4 | - | https://woodbin.com/ref/wood-strength-table/ | handbook | page read |  |
| hardwood: white oak | youngs_modulus | 12.27 | 12.27 | GPa | https://woodbin.com/ref/wood-strength-table/ | handbook | page read | static bending, 1.78 million psi; along the grain |
| hardwood: northern red oak | youngs_modulus | 12.55 | 12.55 | GPa | https://woodbin.com/ref/wood-strength-table/ | handbook | page read | 1.82 million psi |
| hardwood: American beech | youngs_modulus | 11.86 | 11.86 | GPa | https://woodbin.com/ref/wood-strength-table/ | handbook | page read | 1.72 million psi |
| softwood: eastern white pine | youngs_modulus | 8.55 | 8.55 | GPa | https://woodbin.com/ref/wood-strength-table/ | handbook | page read | 1.24 million psi |
| softwood: loblolly pine | youngs_modulus | 12.34 | 12.34 | GPa | https://woodbin.com/ref/wood-strength-table/ | handbook | page read | 1.79 million psi |
| softwood: Sitka spruce | youngs_modulus | 10.83 | 10.83 | GPa | https://woodbin.com/ref/wood-strength-table/ | handbook | page read | 1.57 million psi |
| hardwood: white oak | compressive_strength | 51.3 | 51.3 | MPa | https://woodbin.com/ref/wood-strength-table/ | handbook | page read | parallel to grain, maximum crushing strength, 7440 psi |
| hardwood: northern red oak | compressive_strength | 46.6 | 46.6 | MPa | https://woodbin.com/ref/wood-strength-table/ | handbook | page read | 6760 psi |
| hardwood: American beech | compressive_strength | 50.3 | 50.3 | MPa | https://woodbin.com/ref/wood-strength-table/ | handbook | page read | 7300 psi |
| softwood: eastern white pine | compressive_strength | 33.1 | 33.1 | MPa | https://woodbin.com/ref/wood-strength-table/ | handbook | page read | 4800 psi |
| softwood: loblolly pine | compressive_strength | 49.2 | 49.2 | MPa | https://woodbin.com/ref/wood-strength-table/ | handbook | page read | 7130 psi |
| softwood: Sitka spruce | compressive_strength | 38.7 | 38.7 | MPa | https://woodbin.com/ref/wood-strength-table/ | handbook | page read | 5610 psi |
| hardwood: red oak | poissons_ratio | 0.033 | 0.56 | - | https://www.scribd.com/document/443450144/chapter-05-pdf | handbook | snippet | FPL Wood Handbook Table 5-2, about 12% moisture; six values across the L, R, T planes; column order not confirmed; a mirror copy, check the FPL-GTR-190 PDF |
| hardwood: white oak | poissons_ratio | 0.036 | 0.618 | - | https://www.scribd.com/document/443450144/chapter-05-pdf | handbook | snippet | same table and caveat |
| softwood: Sitka spruce | poissons_ratio | 0.025 | 0.467 | - | https://www.scribd.com/document/443450144/chapter-05-pdf | handbook | snippet | same table and caveat |
| softwood: loblolly pine | poissons_ratio | 0.292 | 0.382 | - | https://www.scribd.com/document/443450144/chapter-05-pdf | handbook | snippet | same table; two columns blank |
| hardwood | tensile_strength |  |  | MPa | none |  |  | no tension value for oak or beech was found in this pass |
| softwood | tensile_strength |  |  | MPa | none |  |  | no tension value for pine or spruce was found in this pass |
| softwood: Scots pine, Norway spruce, European beech | youngs_modulus |  |  | GPa | none |  |  | European species were not in the tables read; Finnish and other studies exist but were not read |

## Item 2. Specific cutting energy and specific cutting force

| material | quantity | low | high | unit | source (link) | source type | read | note |
|---|---|---|---|---|---|---|---|---|
| granite | specific_cutting_energy | 19.71 | 19.71 | J/mm3 | https://hrcak.srce.hr/file/350490 | paper | page read | sample RS1. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21.  |
| granite | specific_cutting_energy | 17.1 | 17.1 | J/mm3 | https://hrcak.srce.hr/file/350490 | paper | page read | sample RS2. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21.  |
| andesite | specific_cutting_energy | 8.36 | 8.36 | J/mm3 | https://hrcak.srce.hr/file/350490 | paper | page read | sample RS3. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21.  |
| gabbro | specific_cutting_energy | 18.9 | 18.9 | J/mm3 | https://hrcak.srce.hr/file/350490 | paper | page read | sample RS5. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21.  |
| granite | specific_cutting_energy | 9.45 | 9.45 | J/mm3 | https://hrcak.srce.hr/file/350490 | paper | page read | sample RS6. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21.  |
| diorite | specific_cutting_energy | 26.45 | 26.45 | J/mm3 | https://hrcak.srce.hr/file/350490 | paper | page read | sample RS7. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21.  |
| granite | specific_cutting_energy | 9.15 | 9.15 | J/mm3 | https://hrcak.srce.hr/file/350490 | paper | page read | sample RS10. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21.  |
| andesite | specific_cutting_energy | 6.07 | 6.07 | J/mm3 | https://hrcak.srce.hr/file/350490 | paper | page read | sample RS11. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21.  |
| igneous rocks (granite, andesite, gabbro, diorite) | specific_cutting_energy | 6.07 | 26.45 | J/mm3 | https://hrcak.srce.hr/file/350490 | paper | page read | range of the 8 measured values in Table 4 (journal p.23). 11 samples were cut but only 8 measured values are tabulated; RS4 granite, RS8 syenite and RS9 diorite have no printed value. Table 7 (p.24) reuses samples 1-3 of Table 4 as test data, so the reported R2 0.9474 is not an independent check. Four granites span 9.15-19.71. SE rises with density, Schimazek abrasivity and P-wave velocity and falls with pullback amperage (Eq. 4, p.22; R2 0.858, RMSE 3.47, Table 8 p.25). Unit J/mm3 from Eq. 1 p.19 and Table 6 p.24. |
| granite | specific_cutting_energy |  |  | J/mm3 | https://hrcak.srce.hr/file/450842 | paper | page read | Korman, Kujundzic, Vrandecic 2023, e-Zbornik special issue, doi 10.47960/2232-9080.2023.SI.13.36 (open access): a review, not a cutting test on diorite, granite and andesite. It gives no specific energy, force, kWh/m2 or wire tension values. It cites Khoshouei 2020 (p.40, PDF p.5). The earlier lead description was wrong. |
| granite | sawing_rate | 2 | 5 | m2/h | https://hrcak.srce.hr/file/450842 | paper | page read | Korman 2023 conclusion, journal p.45: average sawing efficiency in granite quarries, larger in some quarries |
| limestone | sawing_rate | 5 | 7 | m2/h | https://hrcak.srce.hr/file/450842 | paper | page read | Korman 2023 p.40 and p.45, quarry efficiency |
| marble / travertine | sawing_rate | 5 | 12 | m2/h | https://hrcak.srce.hr/file/450842 | paper | page read | Korman 2023 p.40 and p.45, quarry efficiency |
| stone | specific_cutting_energy |  |  | J/mm3 | https://acikkaynak.bilecik.edu.tr/bitstreams/3fe24de0-a388-4e32-a11f-91b60c5fb34d/download | paper | snippet | circular diamond sawing of carbonate rocks (Yurdakul and Akdas lead); regression models, no number seen |
| hard rock (unnamed) | specific_cutting_energy |  |  | J/mm3 | https://pmc.ncbi.nlm.nih.gov/articles/PMC9573887 | paper | page read | circular saw simulation: energy rises with feed speed and blade spacing, falls with rotation speed; values only in a figure |
| granite | specific_cutting_energy |  |  | J/mm3 | https://www.sciencegate.app/document/10.4028/www.scientific.net/msf.471-472.625 | paper | snippet | diamond grinding of granite; energy depends on removal mechanism; no number seen |
| marble | specific_cutting_energy |  |  | J/mm3 | https://dergipark.org.tr/en/download/article-file/4039035 | paper | snippet | marble milling: one white marble about 77% higher than another; no absolute number seen |
| steel-type work (not stone) | specific_cutting_energy | 13.5 | 30 | J/mm3 | https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8945253 | vendor | snippet | patent definition of ultra high removal rate grinding; not stone, a scale reference only |
| hardwood | specific_cutting_force | 20 | 20 | N/mm2 | https://www.protmed.uoradea.ro/facultate/anale/protectia_mediului/2011B/silv/14.%20Galis%20Ioan.pdf | paper | snippet | Galis, sharp edge, 8-10% moisture, 240 mm cutter at 4500 rpm; species not stated in the excerpt |
| hardwood | specific_cutting_force | 150 | 150 | N/mm2 | https://industrialmonitordirect.com/blogs/knowledgebase/wood-milling-force-and-motor-power-calculation-methods | vendor | snippet | unattributed web page, medium hardwood; conflicts with the 20 N/mm2 paper value; treat as unverified |
| softwood | specific_cutting_force | 50 | 80 | N/mm2 | https://industrialmonitordirect.com/blogs/knowledgebase/wood-milling-force-and-motor-power-calculation-methods | vendor | snippet | same page, soft pines; unverified |
| hardwood: oak | specific_cutting_force |  |  | N/mm2 | https://bioresources.cnr.ncsu.edu/resources/an-attempt-at-modelling-of-cutting-forces-in-oak-peripheral-milling/ | paper | page read | Mandic, Porankiewicz, Danon 2015 BioResources 10(3) 5489: pedunculate oak peripheral milling, force model in feed per tooth, depth, rake, density, moisture, hardness; the first 100000 characters held no force values |

## Item 3. Diamond wire tension, speed and force ratios

| material | quantity | low | high | unit | source (link) | source type | read | note |
|---|---|---|---|---|---|---|---|---|
| granite | wire_speed | 20 | 24 | m/s | https://hrcak.srce.hr/file/350490 | paper | page read | Khoshouei 2020, Table 3 journal p.21: lab machine linear speed 20-24 m/s, main motor 30 kW, 36 beads per m, 5 m wire, 0.6 m pulley; tension not given (pullback amperage only) |
| granite | motor_power | 30 | 30 | kW | https://hrcak.srce.hr/file/350490 | paper | page read | Khoshouei 2020, Table 3 journal p.21, lab machine main motor |
| stone (quarry wire saw) | motor_power | 30 | 75 | kW | https://hrcak.srce.hr/file/450842 | paper | page read | Korman 2023 p.37 (PDF p.2), usual drive motor power |
| travertine (via Korman 2023) | wire_tension |  |  | N | https://hrcak.srce.hr/file/450842 | paper | page read | Korman 2023 p.42 reports Almasi et al. 2015 tests with feed motor current 45-65 A over 50-100 m2 cuts; no force in newtons; secondary citation |
| granite | wire_tension | 2000 | 2400 | N | https://link.springer.com/article/10.1007/s00170-021-07577-3 | paper | snippet | Konstanty 2021 Int J Adv Manuf Technol; tension rises about 3.5% of the set value along the cut |
| granite | wire_speed | 24 | 30 | m/s | https://link.springer.com/article/10.1007/s00170-021-07577-3 | paper | snippet | same paper |
| granite | wire_speed | 25 | 35 | m/s | https://www.toolwits.com/info/improve-granite-quarry-cutting-efficiency-103178923.html | vendor | snippet | vendor marketing page |
| granite | wire_speed | 20 | 40 | m/s | https://www.toolwits.com/info/what-is-a-diamond-wire-saw-and-how-does-work-103200646.html | vendor | snippet | second vendor page; conflicts with the paper range |
| granite | bead_normal_force | 3.28 | 3.28 | N | https://link.springer.com/article/10.1007/s00170-021-07577-3 | paper | snippet | per bead, class 1 granite at 0.98 m2/h; mean bead surface pressure about 58 kPa |
| granite | bead_tangential_force | 0.35 | 0.35 | N | https://link.springer.com/article/10.1007/s00170-021-07577-3 | paper | snippet | per bead, friction force, same condition |
| granite | force_ratio | 9.4 | 9.4 | Fn/Ft | https://link.springer.com/article/10.1007/s00170-021-07577-3 | paper | snippet | derived: 3.28 N / 0.35 N; the paper argues wire bead loads are much lower than in circular sawing |
| granite | force_ratio |  |  | Fn/Ft | https://mechanika.ktu.lt/index.php/Mech/article/view/23309/9931 | paper | snippet | circular blade sawing of granite: a strong relation between normal and tangential force, depth of cut the main factor; no ratio number seen |

## Item 4. Robot stiffness

| material | quantity | low | high | unit | source (link) | source type | read | note |
|---|---|---|---|---|---|---|---|---|
| KUKA KR240-2 (PROXY for a KR120-class arm, not a KR120) | joint_stiffness | 360000 | 1409800 | Nm/rad | https://hal.science/hal-00632989 | paper | page read | Dumas, Caro, Garnier, Furet, Robotics and Computer-Integrated Manufacturing 27(4) 881-888, 2011, HAL hal-00632989. Table 2, HAL PDF p.9 (manuscript p.8). Unit is Nm/rad: k1 1409800, k2 400760, k3 935280, k4 360000, k5 370000, k6 380000. The earlier 0.36-1.41 kNm/rad was wrong; the correct unit is MNm/rad (0.36-1.41 MNm/rad). These are ASSUMED simulation inputs, not measurements: k1-k3 are taken from Alici and Shirinzadeh 2005 (ref 24, another robot) and k4-k6 are chosen arbitrarily; the paper tests identification on them and defers experiments to future work. KR240-2 is a larger arm than a KR120, so use only as an order-of-magnitude proxy. |
| KUKA KR120 class | joint_stiffness |  |  | Nm/rad | none |  |  | none found for a KR120; the KR240-2 row above is the proxy; Schneider et al. IROS 2014 and the Robotica 2011 paper are leads |
| UR5 (PROXY for UR10 / UR10e) | joint_stiffness |  |  | Nm/rad | https://webthesis.biblio.polito.it/7150/ | thesis | page read | Testa, Politecnico di Torino MSc 2017/18, 'Experimental identification of the joints stiffness of the UR5 robot arm'. Open PDF read in full (95 PDF pages; thesis page = PDF page minus 4). Stiffness appears only as log-scale plots (Figs 38-41, thesis pp.82-87) spanning roughly 1 to 1e20 Nm/rad across tests, with no tabulated values; the thesis itself says the LX-PA wire potentiometer was not accurate enough and the two methods disagree. No usable number. Measured end-effector deflections with payload are 0.015-0.547 mm (Tables 11-19, pp.81-86) under a wrench of 13.2 N, 13.2 N and 9.336 Nm (Eq. 135, p.81). |
| UR5 (PROXY for UR10 / UR10e) | cartesian_stiffness |  |  | N/mm | https://webthesis.biblio.polito.it/7150/ | thesis | page read | the thesis reports no Cartesian stiffness; a user forum post reports several mm of movement when milling polycarbonate (forum.universal-robots.com/t/stiffness-adjustable/27583) |
| industrial robots in general | stiffness_ratio | 0.01 | 0.01 | x CNC machine | https://arxiv.org/pdf/2101.04992.pdf | paper | snippet | stated as up to 100 times lower than a CNC machine, pose dependent; 0.01 is the inverse |

## Item 2 additions. Specific cutting energy (2026-11-14 pass)

| material | quantity | low | high | unit | source (link) | source type | read | note |
|---|---|---|---|---|---|---|---|---|
| granite (Lotus Red G3768) | specific_energy_disc_saw | 0.415 | 0.415 | J/mm3 | https://doi.org/10.1016/j.jclepro.2021.126922 | paper | page read | Bai et al. 2021, J. Clean. Prod. 300:126922, large-block circular diamond sawing, QJS1600 machine, sandwich silent-core blades R 900-1300 mm, 27 runs. Fitted constant l0 of Psc = l0 x MRR (Eq. 1-2, journal p.5; Table 7 p.9; validation table p.12). The paper gives l0 in kJ/cm3, which equals J/mm3. Psc is spindle power minus idle power (Table 4 p.9). Four validation runs at MRR 18-36.7 cm3/s matched measured energy within 9%. An average-MRR fit on machine power, not a force-based value; the only disc-saw energy number read, so treat it as a lower scale reference. |
| granite (Karakurt 2014) | specific_energy_disc_saw |  |  | J/mm3 | https://link.springer.com/article/10.1007/s11771-014-1949-9 | paper | snippet | Karakurt 2014 J Cent South Univ 21:365-372 (Taguchi, one granite, circular diamond blade): optimum for minimum SE is 25 m/s peripheral speed, 70 cm/min traverse, 2 cm depth, 100 mL/s coolant. The SE value itself was not seen; the Springer page redirects to a sign-in, so it was not opened. |
| marble | specific_energy_milling |  |  | J/mm3 | https://doi.org/10.1007/s00170-011-3717-4 | paper | page read | Turchetta 2012 IJAMT 61:441-448 (Coreno Perlato Royal marble, 20 mm electroplated diamond mill, 3000 rpm, depth 0.1-1.0 mm, feed 200-600 mm/min). The energy and force models are power laws in equivalent chip thickness with fitted constants shown only in figure legends (Figs 4-9, pp.4-6, labels garbled). No tabulated J/mm3. The parent paper Polini and Turchetta 2004 (E235, IJMTM 44:1189) is not in the pipeline and its page was not opened. |
| granite | specific_energy_grinding |  |  | J/mm3 | https://doi.org/10.1016/s0301-679x(01)00039-1 | paper | page read | Xu 2001 Tribol Int 34:419-426, dry grinding of gray granite, resin diamond wheel 305 mm, 43 m/s. Reports net power about 90 W per pass (p.3) and energy partition: about 31% to the granite, about 70% to the wheel (p.5, Fig. 7). No specific energy in J/mm3. The related Xu and Malkin 2001 J Manuf Sci Eng 123 (E232, doi 10.1115/1.1344900) is the lead for that number; it is not in the pipeline and was not opened. |

## Item 3 additions. Force ratios (2026-11-14 pass)

| material | quantity | low | high | unit | source (link) | source type | read | note |
|---|---|---|---|---|---|---|---|---|
| marble (Coreno Perlato Royal, milling) | force_ratio | 2.3 | 4.3 | Fn/Ft | https://doi.org/10.1007/s00170-011-3717-4 | paper | page read | derived from Turchetta 2012 p.6: tangential Fc rises from about 30 to about 70 N and radial Ft from about 70 to about 300 N with tool wear at the largest chip thickness (feed 200 mm/min). Ratio radial/tangential is 70/30 to 300/70. Diamond mill, not a disc. The pairing of the quoted values is by wear stage and is an assumption. |
| granite (Sardinian) | force_ratio_disc | 2.4 | 3.0 | Fz/Fy | https://doi.org/10.1016/j.diamond.2019.107579 | paper | page read | derived from the regression lines of Turchetta and Sorrentino 2019, Diamond Relat Mater 100:107579, Fig. 3 p.4, working stage 0: f 0.10 mm Fy=123.31-1.0275Vc, Fz=286-2.27Vc; f 0.05 mm Fy=69.70-0.6075Vc, Fz=160.75-1.065Vc, Vc 35-65 m/s. Ratio Fz/Fy is 2.36-2.45 at f 0.10 and 2.55-3.03 at f 0.05. Fz is vertical and Fy is the feed direction; the paper does not label them normal or tangential, so the mapping to Fn/Ft is an assumption. Disc 180 mm, 40/50 mesh, 15 kW CNC. Peak values: Fz about 220 N, Fy about 90 N at f 0.10 (p.3). |

## Item 5. Diamond tool wear

| material | quantity | low | high | unit | source (link) | source type | read | note |
|---|---|---|---|---|---|---|---|---|
| granite and marble (not separated) | wear_rate_saw_segment | 0.458 | 1.237 | mm3/s | https://doi.org/10.1016/j.matpr.2021.02.346 | paper | page read | Gupta and Pratap 2021, Mater Today Proc, Table 2 p.5: segment wear volume rate for tip speed 30-40 m/s and spindle 600-700 rpm, nine runs; lowest at 30 m/s and 600 rpm, highest at 40 m/s and 700 rpm. The paper says granite and marble were cut but does not give a stone per run, and gives no cut area per run (4000 sq ft cut in total, p.4), so it cannot be converted to wear per area. |
| granite (Sardinian) | g_ratio |  |  | - | https://doi.org/10.1016/j.diamond.2019.107579 | paper | page read | no G-ratio or wear-per-area figure in the stone papers read. Turchetta 2019 reports grit states instead (p.6-7, Figs 7-12): at 35 m/s about 42-47% intact and 12-13% smoothed grains per 600 cm3 stage; above 45 m/s smoothed grains exceed 30% and the smoothed/intact index rises from about 0.4 to about 1.6 (f 0.05). Speed limit for good grit renewal is 45 m/s. |
| granite, marble | wear_rate_wire |  |  | um/m2 | https://link.springer.com/article/10.1007/s00170-021-07577-3 | paper | snippet | no per-area bead wear value read. A secondary summary of Korman et al. 2023 defines specific bead wear as diameter change per sawn area in um/m2; the Korman 2023 e-Zbornik paper read earlier gives no numbers. |

## Item 6. Polishing

| material | quantity | low | high | unit | source (link) | source type | read | note |
|---|---|---|---|---|---|---|---|---|
| granite (white and black) | polish_speed | 250 | 2000 | rpm | https://doi.org/10.1016/j.procir.2018.08.254 | paper | page read | Songmene et al. 2018 Procedia CIRP 77:139-142, Table 1 p.2: diamond polishing tool 50.8 mm, spindle 250-2000 rpm, feed 254-2032 mm/min, grit 45-1800. Higher speed, higher feed and finer grit gave lower Ra (p.2). Test runs used 1000 rpm and 508 mm/min. |
| granite (white and black) | polish_pressure | 20 | 20 | kPa | https://doi.org/10.1016/j.procir.2018.08.254 | paper | page read | Fig. 2 caption p.2 states pressure 0.2 bar (grit 60, 508 mm/min); it is not defined elsewhere in the paper (assumed tool contact or air pressure). 0.2 bar = 20 kPa. |
| granite (white) | grit_sequence |  |  | - | https://doi.org/10.1016/j.procir.2018.08.254 | paper | page read | range 45 to 1800 grit tested one grit per run, no sequence prescribed. Ra at grit 80, 1000 rpm, 508 mm/min: linear path 1.344 um, arc 1.025 um, spiral 0.982 um (Fig. 13, p.4). Ra per grit is plotted only (Fig. 5, p.3, no numbers in the text). Semi-wet and dry tests; dust data not used here. |
| marble and granite (vendor guide) | grit_sequence |  |  | - | https://www.mkdiamond.com/pdf/Tec_Library/Tile_Stone/Resin_Pad_Guidelines.pdf | vendor | page read | ordered list: 50, 150 or 300 (coarse, remove cut marks), then 500, 1000, 2000, 3000 (fine), then buffing pads. MK Diamond resin pad guide p.1: never skip a grit; water only. Mirror finish: cerium oxide or tin oxide paste on a felt wheel. No separate sequence for marble versus granite, no pad pressure or rpm, no Ra per grit. |
| marble (Han white, robotic) | preston_coefficient |  |  | - | https://www.mdpi.com/2076-3417/15/1/318 | paper | page read | Appl Sci 15:318 (2025), sol-gel flexible diamond tool, 20 um diamond, 100 mm, robot, constant force. The removal model is the Preston equation (Eq. 2, p.9-10) with kp left symbolic; no value, no removal rate, no pressure or rpm in the text read. Result: Ra 0.0233 um and gloss 102.5 (60 deg) robotic, against Ra 0.2843 um and gloss 63.25 by hand (p.16-17). Read from the pipeline Markdown (K4094), not the MDPI page. |
| marble | polish_removal_rate |  |  | um/min | https://www.mdpi.com/2076-3417/15/1/318 | paper | page read | no removal rate in the text of Appl Sci 15:318. The 2024 Huang et al. Appl Sci 14:8314 (six marbles, sol-gel diamond pad) was seen only as an abstract; MDPI refused the fetch. |

## Item 4 addition. Robot stiffness (2026-11-14 pass)

| material | quantity | low | high | unit | source (link) | source type | read | note |
|---|---|---|---|---|---|---|---|---|
| KUKA LBR iiwa 14 R820 (PROXY, a 14 kg cobot; not a UR10 or KR120) | joint_stiffness | 3800 | 42000 | Nm/rad | https://doi.org/10.1016/j.ifacol.2017.08.2108 | paper | page read | 'Task-oriented rigidity optimization for 7 DOF redundant manipulators', IFAC-PapersOnLine 50(1) 2017 (pipeline ID L10099), Table 2 p.3. MEASURED with the joint torque sensors, position control mode: k1-k7 = 4.2e4, 3.96e4, 1.22e4, 2.02e4, 0.8e4, 0.38e4, 0.48e4 Nm/rad. Joint 2 loading and unloading fits gave 3.96e4 and 4.19e4 (Fig. 5). Impedance mode would give lower values. The only measured set found for a collaborative arm; no UR10 or UR10e set found. |

## Missing

- Granite density and UCS, sandstone Poisson ratio, limestone and wood tensile strength: no clean source in this pass.
- European beech, Scots pine and Norway spruce constants: not in the tables read.
- Specific cutting energy in J/mm3 for diamond milling or grinding and circular sawing of stone: leads only, no numbers seen. Wire sawing: 8 laboratory values from Khoshouei 2020 (6.07-26.45 J/mm3); Korman 2023 is a review with no energy values.
- Specific cutting force for wood routing: two conflicting numbers, one paper snippet and one unattributed web page.
- Measured Fn/Ft for diamond disc sawing: no number seen. The wire ratio is derived from per-bead forces in one paper.
- Joint or Cartesian stiffness for UR10, UR10e and KUKA KR120: no measured value found. The only number set is the KR240-2 joint stiffness 0.36-1.41 MNm/rad (Nm/rad, not kNm/rad) from Dumas et al. 2011, which are assumed simulation inputs and a proxy for a KR120-class arm. The UR5 thesis gives only log plots, so there is no UR5 proxy value.

2026-11-14 pass, still missing:

- Specific cutting energy in J/mm3 for diamond milling and grinding of granite, marble, limestone and sandstone: no tabulated value read. Leads not opened: Polini and Turchetta 2004 (E235), Xu and Malkin 2001 (E232), Karakurt 2014 (value behind a sign-in). The only disc-saw value is 0.415 J/mm3 for Lotus Red granite (Bai 2021), a machine-power fit.
- Limestone and sandstone: no energy, force, wear or polishing value at all.
- G-ratio for diamond grinding, wear per cut area for saw segments and for wire: none. The only wear rate is 0.458-1.237 mm3/s (Gupta 2021), stone per run not given.
- Preston coefficient, polish removal rate, pad pressure and Ra per grit for marble: none. Granite polishing gives speed, feed and grit ranges only (Songmene 2018). The grit sequence comes from a vendor guide with no stone split.
- Joint stiffness for a UR10, UR10e or KR120: none. Only a measured KUKA LBR iiwa 14 R820 set (3.8e3 to 4.2e4 Nm/rad), a 14 kg cobot, as a proxy. E001 and E005 gave no joint values; E001 has no pipeline Markdown.
