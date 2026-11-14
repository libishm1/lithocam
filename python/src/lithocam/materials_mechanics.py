# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Sourced mechanics values for stone, wood and robots: elastic constants, strengths, specific cutting energy, wire
tension and speed, bead forces, robot stiffness. Every row names its source, the source type and how it was read;
lithocam never adds a number without one. A row with low and high of None records that nothing was found.

read = "page read": the page text was fetched and read. "snippet": only a search excerpt was seen; check the source
before relying on the number. Rows from search summaries whose exact origin table could not be confirmed say so in
the note. Ranges are for different samples and test methods; they are not comparable across rows.

Full tables: docs/MATERIALS_MECHANICS.md (generated from this data).
"""
from __future__ import annotations

from typing import Dict, List, Optional

# quantity -> item number in docs/MATERIALS_MECHANICS.md
ITEM = {
    "youngs_modulus": 1, "poissons_ratio": 1, "density": 1, "specific_gravity_od": 1, "tensile_strength": 1,
    "compressive_strength": 1, "flexural_strength": 1,
    "specific_cutting_energy": 2, "specific_cutting_force": 2,
    "wire_tension": 3, "wire_speed": 3, "bead_normal_force": 3, "bead_tangential_force": 3, "force_ratio": 3,
    "joint_stiffness": 4, "motor_power": 3, "sawing_rate": 2, "cartesian_stiffness": 4, "stiffness_ratio": 4,
    "specific_energy_milling": 2, "specific_energy_grinding": 2, "specific_energy_disc_saw": 2, "force_ratio_disc": 3,
    "g_ratio": 5, "wear_rate_saw_segment": 5, "wear_rate_wire": 5,
    "preston_coefficient": 6, "polish_removal_rate": 6, "grit_sequence": 6, "polish_pressure": 6, "polish_speed": 6,
}

BAI = "https://doi.org/10.1016/j.jclepro.2021.126922"
TUR19 = "https://doi.org/10.1016/j.diamond.2019.107579"
TUR11 = "https://doi.org/10.1007/s00170-011-3717-4"
GUPTA = "https://doi.org/10.1016/j.matpr.2021.02.346"
XU01 = "https://doi.org/10.1016/s0301-679x(01)00039-1"
SONG = "https://doi.org/10.1016/j.procir.2018.08.254"
APP318 = "https://www.mdpi.com/2076-3417/15/1/318"
MKD = "https://www.mkdiamond.com/pdf/Tec_Library/Tile_Stone/Resin_Pad_Guidelines.pdf"
L10099 = "https://doi.org/10.1016/j.ifacol.2017.08.2108"
KARAK = "https://link.springer.com/article/10.1007/s11771-014-1949-9"

GEO = "geotechnical numbers table (tu-freiberg.de Useful Numbers PDF or sites.psu.edu/tzhu copy; which one was not confirmed)"
GEO_URL = "https://tu-freiberg.de/sites/default/files/2023-11/56%20Useful%20Numbers%20for%20Geotechnical%20Engineers%206.pdf"
WOODBIN = "https://woodbin.com/ref/wood-strength-table/"
FPL5 = "https://www.scribd.com/document/443450144/chapter-05-pdf"
KON = "https://link.springer.com/article/10.1007/s00170-021-07577-3"
SPOT = "https://spotmaterial.com/material/marble/slab/"
STONELINE = "https://stoneline.com.tr/en/?p=28479"
POLY = "https://www.polycor.com/stones/indiana-limestone-standard-buff/"
SEL = "https://www.selectstone.com/?p=8110"
WANG = "https://pmc.ncbi.nlm.nih.gov/articles/PMC9573887"
DUMAS = "https://hal.science/hal-00632989"
KHO = "https://hrcak.srce.hr/file/350490"
KORMAN = "https://hrcak.srce.hr/file/450842"
POLITO = "https://webthesis.biblio.polito.it/7150/"
TOOLWITS = "https://www.toolwits.com/info/improve-granite-quarry-cutting-efficiency-103178923.html"


def _r(material, quantity, low, high, unit, source, source_type, read, note=""):
    return dict(material=material, quantity=quantity, low=low, high=high, unit=unit, source=source,
                source_type=source_type, read=read, note=note)


def _g(material, quantity, low, high, unit, note=""):
    return _r(material, quantity, low, high, unit, GEO_URL, "handbook table", "snippet",
              "search summary of a geotechnical table, origin not confirmed. " + note)


TABLE: List[Dict] = [
    # ---- item 1: rock, generic ranges
    _g("sandstone", "youngs_modulus", 5, 50, "GPa"),
    _g("limestone", "youngs_modulus", 15, 70, "GPa"),
    _g("marble", "youngs_modulus", 30, 70, "GPa", "an older edition of such a table gave 35-60 GPa"),
    _g("granite", "youngs_modulus", 20, 70, "GPa"),
    _g("sandstone", "poissons_ratio", 0.2, 0.4, "-"),
    _g("limestone", "poissons_ratio", 0.2, 0.35, "-"),
    _g("granite", "poissons_ratio", 0.1, 0.3, "-", "typical 0.1-0.2"),
    _g("marble", "poissons_ratio", 0.1, 0.3, "-", "a separate table gave 0.06-0.22"),
    _g("granite", "tensile_strength", 6, 15, "MPa", "test method not stated in the summary"),
    _g("sandstone", "tensile_strength", 1, 15, "MPa", "another table gave 4-25"),
    _g("limestone", "tensile_strength", 10, 20, "MPa", "another table gave 5-25"),
    _g("marble", "tensile_strength", 5, 10, "MPa"),
    # ---- item 1: rock, specific stones
    _r("marble", "density", 2.7, 2.7, "g/cm3", STONELINE, "vendor", "snippet", "Carrara datasheet"),
    _r("marble", "density", 2500, 2700, "kg/m3", SPOT, "vendor", "snippet", "marble in general, materials database"),
    _r("marble", "poissons_ratio", 0.2, 0.2, "-", SPOT, "vendor", "snippet", "marble in general, materials database"),
    _r("marble", "youngs_modulus", 21.53, 21.53, "GPa",
       "https://airdrive.eventsair.com/eventsairwesteuprod/production-atlanticmice-public/8f7fdf2218664bc2874fd37f40e7d4d5",
       "paper", "snippet", "Carrara, mean tangent modulus from a conference study, standard deviation 8.62 GPa"),
    _r("marble", "compressive_strength", 75, 75, "MPa", STONELINE, "vendor", "snippet", "Carrara datasheet, 10930 psi"),
    _r("marble", "compressive_strength", 100, 100, "MPa", STONELINE, "vendor", "snippet",
       "a second Carrara listing gave 1020 kg/cm2, about 100 MPa; origin page not confirmed"),
    _r("marble", "compressive_strength", 33, 210, "MPa", SPOT, "vendor", "snippet", "marble in general"),
    _r("marble", "tensile_strength", 2.4, 24.5, "MPa", SPOT, "vendor", "snippet",
       "marble in general; test method not stated; no Carrara Brazilian value found"),
    _r("limestone", "density", 2307, 2307, "kg/m3", POLY, "vendor", "snippet",
       "Indiana limestone, 144 lb/ft3; the Indiana Limestone Institute minimum for medium density is 135 lb/ft3"),
    _r("limestone", "compressive_strength", 27.58, 27.58, "MPa", POLY, "vendor", "snippet",
       "Indiana limestone, 4000 psi, a specified minimum; most products exceed it"),
    _r("limestone", "flexural_strength", 4.83, 4.83, "MPa", POLY, "vendor", "snippet",
       "Indiana limestone modulus of rupture, 700 psi; flexural, not direct tension"),
    _r("sandstone", "density", 2147, 2147, "kg/m3", SEL, "vendor", "snippet", "Berea sandstone, 134 lb/ft3"),
    _r("sandstone", "compressive_strength", 79.8, 79.8, "MPa", SEL, "vendor", "snippet", "Berea sandstone, 11576 psi"),
    _r("hard rock (unnamed)", "density", 2564.76, 2564.76, "kg/m3", WANG, "paper", "page read",
       "Wang et al. 2022 Sci Rep 12:17346, Table 1; the model itself used 2670 kg/m3"),
    _r("hard rock (unnamed)", "compressive_strength", 108.45, 108.45, "MPa", WANG, "paper", "page read",
       "same table; rock type not named"),
    _r("hard rock (unnamed)", "tensile_strength", 6.67, 6.67, "MPa", WANG, "paper", "page read", "same table"),
    _r("hard rock (unnamed)", "poissons_ratio", 0.209, 0.209, "-", WANG, "paper", "page read", "same table"),
    _r("granite", "density", None, None, "kg/m3", "", "", "", "no clean source found in this pass"),
    _r("granite", "compressive_strength", None, None, "MPa", "", "", "",
       "no clean source found in this pass; Kendrick et al. 2023 Adv. Geosci. 62 (G603 granite, UCS and Brazilian) is "
       "the lead, not read"),
    _r("sandstone", "poissons_ratio", None, None, "-", "", "", "", "only the generic range above was found"),
    _r("limestone", "tensile_strength", None, None, "MPa", "", "", "",
       "no Brazilian value for a named limestone found; only the generic range above"),
    # ---- item 1: wood (FPL values for US species; woodbin page attributes them to the Forest Products Laboratory)
    _r("hardwood: white oak", "specific_gravity_od", 0.68, 0.68, "-", WOODBIN, "handbook", "page read",
       "oven-dry specific gravity; the page does not state the moisture basis of the other columns"),
    _r("hardwood: northern red oak", "specific_gravity_od", 0.63, 0.63, "-", WOODBIN, "handbook", "page read", ""),
    _r("hardwood: American beech", "specific_gravity_od", 0.64, 0.64, "-", WOODBIN, "handbook", "page read", ""),
    _r("softwood: eastern white pine", "specific_gravity_od", 0.35, 0.35, "-", WOODBIN, "handbook", "page read", ""),
    _r("softwood: loblolly pine", "specific_gravity_od", 0.51, 0.51, "-", WOODBIN, "handbook", "page read", ""),
    _r("softwood: Sitka spruce", "specific_gravity_od", 0.40, 0.40, "-", WOODBIN, "handbook", "page read", ""),
    _r("hardwood: white oak", "youngs_modulus", 12.27, 12.27, "GPa", WOODBIN, "handbook", "page read",
       "static bending, 1.78 million psi; along the grain"),
    _r("hardwood: northern red oak", "youngs_modulus", 12.55, 12.55, "GPa", WOODBIN, "handbook", "page read",
       "1.82 million psi"),
    _r("hardwood: American beech", "youngs_modulus", 11.86, 11.86, "GPa", WOODBIN, "handbook", "page read",
       "1.72 million psi"),
    _r("softwood: eastern white pine", "youngs_modulus", 8.55, 8.55, "GPa", WOODBIN, "handbook", "page read",
       "1.24 million psi"),
    _r("softwood: loblolly pine", "youngs_modulus", 12.34, 12.34, "GPa", WOODBIN, "handbook", "page read",
       "1.79 million psi"),
    _r("softwood: Sitka spruce", "youngs_modulus", 10.83, 10.83, "GPa", WOODBIN, "handbook", "page read",
       "1.57 million psi"),
    _r("hardwood: white oak", "compressive_strength", 51.3, 51.3, "MPa", WOODBIN, "handbook", "page read",
       "parallel to grain, maximum crushing strength, 7440 psi"),
    _r("hardwood: northern red oak", "compressive_strength", 46.6, 46.6, "MPa", WOODBIN, "handbook", "page read",
       "6760 psi"),
    _r("hardwood: American beech", "compressive_strength", 50.3, 50.3, "MPa", WOODBIN, "handbook", "page read",
       "7300 psi"),
    _r("softwood: eastern white pine", "compressive_strength", 33.1, 33.1, "MPa", WOODBIN, "handbook", "page read",
       "4800 psi"),
    _r("softwood: loblolly pine", "compressive_strength", 49.2, 49.2, "MPa", WOODBIN, "handbook", "page read",
       "7130 psi"),
    _r("softwood: Sitka spruce", "compressive_strength", 38.7, 38.7, "MPa", WOODBIN, "handbook", "page read",
       "5610 psi"),
    _r("hardwood: red oak", "poissons_ratio", 0.033, 0.560, "-", FPL5, "handbook", "snippet",
       "FPL Wood Handbook Table 5-2, about 12% moisture; six values across the L, R, T planes; column order not "
       "confirmed; a mirror copy, check the FPL-GTR-190 PDF"),
    _r("hardwood: white oak", "poissons_ratio", 0.036, 0.618, "-", FPL5, "handbook", "snippet", "same table and caveat"),
    _r("softwood: Sitka spruce", "poissons_ratio", 0.025, 0.467, "-", FPL5, "handbook", "snippet", "same table and caveat"),
    _r("softwood: loblolly pine", "poissons_ratio", 0.292, 0.382, "-", FPL5, "handbook", "snippet",
       "same table; two columns blank"),
    _r("hardwood", "tensile_strength", None, None, "MPa", "", "", "",
       "no tension value for oak or beech was found in this pass"),
    _r("softwood", "tensile_strength", None, None, "MPa", "", "", "",
       "no tension value for pine or spruce was found in this pass"),
    _r("softwood: Scots pine, Norway spruce, European beech", "youngs_modulus", None, None, "GPa", "", "", "",
       "European species were not in the tables read; Finnish and other studies exist but were not read"),
    # ---- item 2: specific cutting energy and cutting force
    _r("granite", "specific_cutting_energy", 19.71, 19.71, "J/mm3", KHO, "paper", "page read",
       "sample RS1. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21. "),
    _r("granite", "specific_cutting_energy", 17.1, 17.1, "J/mm3", KHO, "paper", "page read",
       "sample RS2. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21. "),
    _r("andesite", "specific_cutting_energy", 8.36, 8.36, "J/mm3", KHO, "paper", "page read",
       "sample RS3. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21. "),
    _r("gabbro", "specific_cutting_energy", 18.9, 18.9, "J/mm3", KHO, "paper", "page read",
       "sample RS5. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21. "),
    _r("granite", "specific_cutting_energy", 9.45, 9.45, "J/mm3", KHO, "paper", "page read",
       "sample RS6. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21. "),
    _r("diorite", "specific_cutting_energy", 26.45, 26.45, "J/mm3", KHO, "paper", "page read",
       "sample RS7. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21. "),
    _r("granite", "specific_cutting_energy", 9.15, 9.15, "J/mm3", KHO, "paper", "page read",
       "sample RS10. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21. "),
    _r("andesite", "specific_cutting_energy", 6.07, 6.07, "J/mm3", KHO, "paper", "page read",
       "sample RS11. Khoshouei, Jalalian, Bagherpour 2020, Rudarsko-geolosko-naftni zbornik 35(3) 17-27, doi 10.17794/rgn.2020.3.2 (open access), Table 4, journal p.23 (PDF p.7). Lab diamond wire saw, 30 kW, 20-24 m/s. Energy = motor power x cutting time / (cut length x height x bead thickness), p.21. "),
    _r("igneous rocks (granite, andesite, gabbro, diorite)", "specific_cutting_energy", 6.07, 26.45, "J/mm3", KHO, "paper", "page read",
       "range of the 8 measured values in Table 4 (journal p.23). 11 samples were cut but only 8 measured values are tabulated; "
       "RS4 granite, RS8 syenite and RS9 diorite have no printed value. Table 7 (p.24) reuses samples 1-3 of Table 4 as test data, "
       "so the reported R2 0.9474 is not an independent check. Four granites span 9.15-19.71. SE rises with density, Schimazek abrasivity "
       "and P-wave velocity and falls with pullback amperage (Eq. 4, p.22; R2 0.858, RMSE 3.47, Table 8 p.25). Unit J/mm3 from Eq. 1 p.19 and Table 6 p.24."),
    _r("granite", "wire_speed", 20, 24, "m/s", KHO, "paper", "page read",
       "Khoshouei 2020, Table 3 journal p.21: lab machine linear speed 20-24 m/s, main motor 30 kW, 36 beads per m, 5 m wire, 0.6 m pulley; tension not given (pullback amperage only)"),
    _r("granite", "motor_power", 30, 30, "kW", KHO, "paper", "page read", "Khoshouei 2020, Table 3 journal p.21, lab machine main motor"),
    _r("granite", "specific_cutting_energy", None, None, "J/mm3", KORMAN, "paper", "page read",
       "Korman, Kujundzic, Vrandecic 2023, e-Zbornik special issue, doi 10.47960/2232-9080.2023.SI.13.36 (open access): a review, not a cutting test on diorite, granite and andesite. "
       "It gives no specific energy, force, kWh/m2 or wire tension values. It cites Khoshouei 2020 (p.40, PDF p.5). The earlier lead description was wrong."),
    _r("granite", "sawing_rate", 2, 5, "m2/h", KORMAN, "paper", "page read",
       "Korman 2023 conclusion, journal p.45: average sawing efficiency in granite quarries, larger in some quarries"),
    _r("limestone", "sawing_rate", 5, 7, "m2/h", KORMAN, "paper", "page read", "Korman 2023 p.40 and p.45, quarry efficiency"),
    _r("marble / travertine", "sawing_rate", 5, 12, "m2/h", KORMAN, "paper", "page read", "Korman 2023 p.40 and p.45, quarry efficiency"),
    _r("stone (quarry wire saw)", "motor_power", 30, 75, "kW", KORMAN, "paper", "page read", "Korman 2023 p.37 (PDF p.2), usual drive motor power"),
    _r("travertine (via Korman 2023)", "wire_tension", None, None, "N", KORMAN, "paper", "page read",
       "Korman 2023 p.42 reports Almasi et al. 2015 tests with feed motor current 45-65 A over 50-100 m2 cuts; no force in newtons; secondary citation"),
    _r("stone", "specific_cutting_energy", None, None, "J/mm3", "https://acikkaynak.bilecik.edu.tr/bitstreams/3fe24de0-a388-4e32-a11f-91b60c5fb34d/download",
       "paper", "snippet",
       "circular diamond sawing of carbonate rocks (Yurdakul and Akdas lead); regression models, no number seen"),
    _r("hard rock (unnamed)", "specific_cutting_energy", None, None, "J/mm3", WANG, "paper", "page read",
       "circular saw simulation: energy rises with feed speed and blade spacing, falls with rotation speed; values "
       "only in a figure"),
    _r("granite", "specific_cutting_energy", None, None, "J/mm3", "https://www.sciencegate.app/document/10.4028/www.scientific.net/msf.471-472.625",
       "paper", "snippet", "diamond grinding of granite; energy depends on removal mechanism; no number seen"),
    _r("marble", "specific_cutting_energy", None, None, "J/mm3", "https://dergipark.org.tr/en/download/article-file/4039035",
       "paper", "snippet", "marble milling: one white marble about 77% higher than another; no absolute number seen"),
    _r("steel-type work (not stone)", "specific_cutting_energy", 13.5, 30, "J/mm3",
       "https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8945253", "vendor", "snippet",
       "patent definition of ultra high removal rate grinding; not stone, a scale reference only"),
    _r("hardwood", "specific_cutting_force", 20, 20, "N/mm2", "https://www.protmed.uoradea.ro/facultate/anale/protectia_mediului/2011B/silv/14.%20Galis%20Ioan.pdf",
       "paper", "snippet", "Galis, sharp edge, 8-10% moisture, 240 mm cutter at 4500 rpm; species not stated in the excerpt"),
    _r("hardwood", "specific_cutting_force", 150, 150, "N/mm2",
       "https://industrialmonitordirect.com/blogs/knowledgebase/wood-milling-force-and-motor-power-calculation-methods",
       "vendor", "snippet", "unattributed web page, medium hardwood; conflicts with the 20 N/mm2 paper value; treat as unverified"),
    _r("softwood", "specific_cutting_force", 50, 80, "N/mm2",
       "https://industrialmonitordirect.com/blogs/knowledgebase/wood-milling-force-and-motor-power-calculation-methods",
       "vendor", "snippet", "same page, soft pines; unverified"),
    _r("hardwood: oak", "specific_cutting_force", None, None, "N/mm2",
       "https://bioresources.cnr.ncsu.edu/resources/an-attempt-at-modelling-of-cutting-forces-in-oak-peripheral-milling/",
       "paper", "page read",
       "Mandic, Porankiewicz, Danon 2015 BioResources 10(3) 5489: pedunculate oak peripheral milling, force model in "
       "feed per tooth, depth, rake, density, moisture, hardness; the first 100000 characters held no force values"),
    # ---- item 3: diamond wire and disc forces
    _r("granite", "wire_tension", 2000, 2400, "N", KON, "paper", "snippet",
       "Konstanty 2021 Int J Adv Manuf Technol; tension rises about 3.5% of the set value along the cut"),
    _r("granite", "wire_speed", 24, 30, "m/s", KON, "paper", "snippet", "same paper"),
    _r("granite", "wire_speed", 25, 35, "m/s", TOOLWITS, "vendor", "snippet", "vendor marketing page"),
    _r("granite", "wire_speed", 20, 40, "m/s", "https://www.toolwits.com/info/what-is-a-diamond-wire-saw-and-how-does-work-103200646.html",
       "vendor", "snippet", "second vendor page; conflicts with the paper range"),
    _r("granite", "bead_normal_force", 3.28, 3.28, "N", KON, "paper", "snippet",
       "per bead, class 1 granite at 0.98 m2/h; mean bead surface pressure about 58 kPa"),
    _r("granite", "bead_tangential_force", 0.35, 0.35, "N", KON, "paper", "snippet",
       "per bead, friction force, same condition"),
    _r("granite", "force_ratio", 9.4, 9.4, "Fn/Ft", KON, "paper", "snippet",
       "derived: 3.28 N / 0.35 N; the paper argues wire bead loads are much lower than in circular sawing"),
    _r("granite", "force_ratio", None, None, "Fn/Ft", "https://mechanika.ktu.lt/index.php/Mech/article/view/23309/9931",
       "paper", "snippet",
       "circular blade sawing of granite: a strong relation between normal and tangential force, depth of cut the "
       "main factor; no ratio number seen"),
    # ---- 2026-11-14 pass: papers read through a local literature pipeline, plus open pages. Numbers only.
    _r("granite (Lotus Red G3768)", "specific_energy_disc_saw", 0.415, 0.415, "J/mm3", BAI, "paper", "page read",
       "Bai et al. 2021, J. Clean. Prod. 300:126922, large-block circular diamond sawing, QJS1600 machine, sandwich silent-core blades R 900-1300 mm, 27 runs. "
       "Fitted constant l0 of Psc = l0 x MRR (Eq. 1-2, journal p.5; Table 7 p.9; validation table p.12). The paper gives l0 in kJ/cm3, which equals J/mm3. "
       "Psc is spindle power minus idle power (Table 4 p.9). Four validation runs at MRR 18-36.7 cm3/s matched measured energy within 9%. "
       "An average-MRR fit on machine power, not a force-based value; the only disc-saw energy number read, so treat it as a lower scale reference."),
    _r("granite (Karakurt 2014)", "specific_energy_disc_saw", None, None, "J/mm3", KARAK, "paper", "snippet",
       "Karakurt 2014 J Cent South Univ 21:365-372 (Taguchi, one granite, circular diamond blade): optimum for minimum SE is 25 m/s peripheral speed, "
       "70 cm/min traverse, 2 cm depth, 100 mL/s coolant. The SE value itself was not seen; the Springer page redirects to a sign-in, so it was not opened."),
    _r("marble", "specific_energy_milling", None, None, "J/mm3", TUR11, "paper", "page read",
       "Turchetta 2012 IJAMT 61:441-448 (Coreno Perlato Royal marble, 20 mm electroplated diamond mill, 3000 rpm, depth 0.1-1.0 mm, feed 200-600 mm/min). "
       "The energy and force models are power laws in equivalent chip thickness with fitted constants shown only in figure legends (Figs 4-9, pp.4-6, labels garbled). "
       "No tabulated J/mm3. The parent paper Polini and Turchetta 2004 (E235, IJMTM 44:1189) is not in the pipeline and its page was not opened."),
    _r("granite", "specific_energy_grinding", None, None, "J/mm3", XU01, "paper", "page read",
       "Xu 2001 Tribol Int 34:419-426, dry grinding of gray granite, resin diamond wheel 305 mm, 43 m/s. Reports net power about 90 W per pass (p.3) and energy partition: "
       "about 31% to the granite, about 70% to the wheel (p.5, Fig. 7). No specific energy in J/mm3. The related Xu and Malkin 2001 J Manuf Sci Eng 123 (E232, doi 10.1115/1.1344900) "
       "is the lead for that number; it is not in the pipeline and was not opened."),
    _r("marble (Coreno Perlato Royal, milling)", "force_ratio", 2.3, 4.3, "Fn/Ft", TUR11, "paper", "page read",
       "derived from Turchetta 2012 p.6: tangential Fc rises from about 30 to about 70 N and radial Ft from about 70 to about 300 N with tool wear at the largest chip thickness "
       "(feed 200 mm/min). Ratio radial/tangential is 70/30 to 300/70. Diamond mill, not a disc. The pairing of the quoted values is by wear stage and is an assumption."),
    _r("granite (Sardinian)", "force_ratio_disc", 2.4, 3.0, "Fz/Fy", TUR19, "paper", "page read",
       "derived from the regression lines of Turchetta and Sorrentino 2019, Diamond Relat Mater 100:107579, Fig. 3 p.4, working stage 0: f 0.10 mm Fy=123.31-1.0275Vc, Fz=286-2.27Vc; "
       "f 0.05 mm Fy=69.70-0.6075Vc, Fz=160.75-1.065Vc, Vc 35-65 m/s. Ratio Fz/Fy is 2.36-2.45 at f 0.10 and 2.55-3.03 at f 0.05. Fz is vertical and Fy is the feed direction; "
       "the paper does not label them normal or tangential, so the mapping to Fn/Ft is an assumption. Disc 180 mm, 40/50 mesh, 15 kW CNC. Peak values: Fz about 220 N, Fy about 90 N at f 0.10 (p.3)."),
    _r("granite and marble (not separated)", "wear_rate_saw_segment", 0.458, 1.237, "mm3/s", GUPTA, "paper", "page read",
       "Gupta and Pratap 2021, Mater Today Proc, Table 2 p.5: segment wear volume rate for tip speed 30-40 m/s and spindle 600-700 rpm, nine runs; lowest at 30 m/s and 600 rpm, highest at 40 m/s and 700 rpm. "
       "The paper says granite and marble were cut but does not give a stone per run, and gives no cut area per run (4000 sq ft cut in total, p.4), so it cannot be converted to wear per area."),
    _r("granite (Sardinian)", "g_ratio", None, None, "-", TUR19, "paper", "page read",
       "no G-ratio or wear-per-area figure in the stone papers read. Turchetta 2019 reports grit states instead (p.6-7, Figs 7-12): at 35 m/s about 42-47% intact and 12-13% smoothed grains per 600 cm3 stage; "
       "above 45 m/s smoothed grains exceed 30% and the smoothed/intact index rises from about 0.4 to about 1.6 (f 0.05). Speed limit for good grit renewal is 45 m/s."),
    _r("granite, marble", "wear_rate_wire", None, None, "um/m2", "https://link.springer.com/article/10.1007/s00170-021-07577-3", "paper", "snippet",
       "no per-area bead wear value read. A secondary summary of Korman et al. 2023 defines specific bead wear as diameter change per sawn area in um/m2; the Korman 2023 e-Zbornik paper read earlier gives no numbers."),
    _r("granite (white and black)", "polish_speed", 250, 2000, "rpm", SONG, "paper", "page read",
       "Songmene et al. 2018 Procedia CIRP 77:139-142, Table 1 p.2: diamond polishing tool 50.8 mm, spindle 250-2000 rpm, feed 254-2032 mm/min, grit 45-1800. "
       "Higher speed, higher feed and finer grit gave lower Ra (p.2). Test runs used 1000 rpm and 508 mm/min."),
    _r("granite (white and black)", "polish_pressure", 20, 20, "kPa", SONG, "paper", "page read",
       "Fig. 2 caption p.2 states pressure 0.2 bar (grit 60, 508 mm/min); it is not defined elsewhere in the paper (assumed tool contact or air pressure). 0.2 bar = 20 kPa."),
    _r("granite (white)", "grit_sequence", None, None, "-", SONG, "paper", "page read",
       "range 45 to 1800 grit tested one grit per run, no sequence prescribed. Ra at grit 80, 1000 rpm, 508 mm/min: linear path 1.344 um, arc 1.025 um, spiral 0.982 um (Fig. 13, p.4). "
       "Ra per grit is plotted only (Fig. 5, p.3, no numbers in the text). Semi-wet and dry tests; dust data not used here."),
    _r("marble and granite (vendor guide)", "grit_sequence", None, None, "-", MKD, "vendor", "page read",
       "ordered list: 50, 150 or 300 (coarse, remove cut marks), then 500, 1000, 2000, 3000 (fine), then buffing pads. MK Diamond resin pad guide p.1: never skip a grit; water only. "
       "Mirror finish: cerium oxide or tin oxide paste on a felt wheel. No separate sequence for marble versus granite, no pad pressure or rpm, no Ra per grit."),
    _r("marble (Han white, robotic)", "preston_coefficient", None, None, "-", APP318, "paper", "page read",
       "Appl Sci 15:318 (2025), sol-gel flexible diamond tool, 20 um diamond, 100 mm, robot, constant force. The removal model is the Preston equation (Eq. 2, p.9-10) with kp "
       "left symbolic; no value, no removal rate, no pressure or rpm in the text read. Result: Ra 0.0233 um and gloss 102.5 (60 deg) robotic, against Ra 0.2843 um and gloss 63.25 by hand (p.16-17). "
       "Read from the pipeline Markdown (K4094), not the MDPI page."),
    _r("marble", "polish_removal_rate", None, None, "um/min", APP318, "paper", "page read",
       "no removal rate in the text of Appl Sci 15:318. The 2024 Huang et al. Appl Sci 14:8314 (six marbles, sol-gel diamond pad) was seen only as an abstract; MDPI refused the fetch."),
    # ---- item 4: robot stiffness
    _r("KUKA LBR iiwa 14 R820 (PROXY, a 14 kg cobot)", "joint_stiffness", 3800, 42000, "Nm/rad", L10099, "paper", "page read",
       "'Task-oriented rigidity optimization for 7 DOF redundant manipulators', IFAC-PapersOnLine 50(1) 2017 (pipeline ID L10099), Table 2 p.3. MEASURED with the joint torque sensors, position control mode: "
       "k1-k7 = 4.2e4, 3.96e4, 1.22e4, 2.02e4, 0.8e4, 0.38e4, 0.48e4 Nm/rad. Joint 2 loading and unloading fits gave 3.96e4 and 4.19e4 (Fig. 5). Impedance mode would give lower values. "
       "The only measured set found for a collaborative arm; no UR10 or UR10e set found."),
    _r("KUKA KR240-2 (PROXY for a KR120-class arm, not a KR120)", "joint_stiffness", 360000, 1409800, "Nm/rad", DUMAS, "paper", "page read",
       "Dumas, Caro, Garnier, Furet, Robotics and Computer-Integrated Manufacturing 27(4) 881-888, 2011, HAL hal-00632989. Table 2, HAL PDF p.9 (manuscript p.8). "
       "Unit is Nm/rad: k1 1409800, k2 400760, k3 935280, k4 360000, k5 370000, k6 380000. The earlier 0.36-1.41 kNm/rad was wrong; the correct unit is "
       "MNm/rad (0.36-1.41 MNm/rad). These are ASSUMED simulation inputs, not measurements: k1-k3 are taken from Alici and Shirinzadeh 2005 (ref 24, another robot) and "
       "k4-k6 are chosen arbitrarily; the paper tests identification on them and defers experiments to future work. KR240-2 is a larger arm than a KR120, "
       "so use only as an order-of-magnitude proxy."),
    _r("KUKA KR120 class", "joint_stiffness", None, None, "Nm/rad", "", "", "",
       "none found for a KR120; the KR240-2 row above is the proxy; Schneider et al. IROS 2014 and the Robotica 2011 paper are leads"),
    _r("UR5 (PROXY for UR10 / UR10e)", "joint_stiffness", None, None, "Nm/rad", POLITO, "thesis", "page read",
       "Testa, Politecnico di Torino MSc 2017/18, 'Experimental identification of the joints stiffness of the UR5 robot arm'. Open PDF read in full (95 PDF pages; "
       "thesis page = PDF page minus 4). Stiffness appears only as log-scale plots (Figs 38-41, thesis pp.82-87) spanning roughly 1 to 1e20 Nm/rad across tests, "
       "with no tabulated values; the thesis itself says the LX-PA wire potentiometer was not accurate enough and the two methods disagree. No usable number. "
       "Measured end-effector deflections with payload are 0.015-0.547 mm (Tables 11-19, pp.81-86) under a wrench of 13.2 N, 13.2 N and 9.336 Nm (Eq. 135, p.81)."),
    _r("UR5 (PROXY for UR10 / UR10e)", "cartesian_stiffness", None, None, "N/mm", POLITO, "thesis", "page read",
       "the thesis reports no Cartesian stiffness; a user forum post reports several mm of movement when milling polycarbonate "
       "(forum.universal-robots.com/t/stiffness-adjustable/27583)"),
    _r("industrial robots in general", "stiffness_ratio", 0.01, 0.01, "x CNC machine", "https://arxiv.org/pdf/2101.04992.pdf",
       "paper", "snippet", "stated as up to 100 times lower than a CNC machine, pose dependent; 0.01 is the inverse"),
    # ---- 2026-11-14 constants review (outputs of the review are kept outside the repo; every row names its locator)
    _r("KUKA KR210 R2700 (PROXY for a KR120-class arm)", "joint_stiffness", 1.01e5, 3.24e6, "Nm/rad",
       "https://doi.org/10.1016/j.rcim.2020.101953", "paper", "page read",
       "Table 1: k1-k6 = 1.89e6, 2.89e6, 3.24e6, 1.01e5, 1.38e5, 1.28e5 Nm/rad, identified with a laser tracker. A larger arm "
       "than the KR120 R2500, so a proxy; the wrist joints (k4-k6) set the low end."),
    _r("KUKA KR 500-3 (PROXY for the KR 480 / KR 500 MT cells)", "joint_stiffness", 1.4e5, 2.86e6, "Nm/rad",
       "https://doi.org/10.1016/j.rcim.2020.101992", "paper", "page read",
       "Cvitanic et al. 2020, Table 2, model 2: 2.86, 2.36, 1.67, 0.24, 0.16, 0.14 MNm/rad (model 1: 3.93, 3.46, 2.77, 1.57, "
       "0.24, 0.17). Laser-tracker identification of a KR 500-3, not the MT variants."),
    _r("ABB IRB 6660 (PROXY for an IRB 6700-class arm)", "joint_stiffness", 4e5, 2e6, "Nm/rad",
       "corpus K294 Table II and L1178 Table 1 (Subrin et al., Dumas method)", "paper", "page read",
       "k1-k6 = 1e6, 2e6, 2e6, 4e5, 4e5, 4e5 Nm/rad; two papers agree. The IRB 6660 is a stiffer parallel-arm machine, so "
       "this proxy is optimistic for an IRB 6700."),
    _r("bluestone (lithology not stated), robotic grinding", "specific_cutting_energy", 0.277, 0.385, "J/mm3",
       "https://doi.org/10.1038/s41598-025-92941-6", "paper", "page read",
       "Cheng et al. 2025 Sci Rep 15:8544, Tables 8-9: u = 60 P/(ae ap f) over six runs (KUKA KR240-2900, ae 10-25 mm, ap 1-2 mm, "
       "f 3000-4800 mm/min), P = spindle minus no-load power. Derived here; the paper's power fit gives D = 50.39 mm for all six "
       "runs, which checks the pairing."),
    _r("granite, black Jinan (sintered diamond mill)", "g_ratio", 5032, 7716, "mm3/mm3",
       "https://doi.org/10.4028/www.scientific.net/MSF.471-472.469", "paper", "page read",
       "Huang et al. 2004 Table 1: mill M2, 54 mm, 12 sectors 46x8x4 mm, about 4.5 mm diameter loss over about 50000 cm3. Derived "
       "from diameter loss, removed volume and segment size; the text also says about 5 mm for both mills, so low confidence."),
    _r("granite, red Shidao (sintered diamond mill)", "g_ratio", 15851, 29167, "mm3/mm3",
       "https://doi.org/10.4028/www.scientific.net/MSF.471-472.469", "paper", "page read",
       "Huang et al. 2004 Table 1: mill M1, 59 mm, 16 sectors 46x6x4 mm, about 4 mm diameter loss over about 140000 cm3. Derived; "
       "low confidence, as the black Jinan row."),
    _r("granite (nine Indian granites, multi-blade circular saw)", "wear_per_area", 0.005069, 0.013462, "mm/m2",
       "https://doi.org/10.1038/s41598-024-54625-5", "paper", "page read",
       "Rajpurohit et al. 2024 Sci Rep 14:4590, Table 2, printed as 5.069-13.462 um/m2: Fe-based multilayer segments, 40/50 mesh, "
       "343 rpm, traverse 50-100 mm/s, 20-25 mm per pass. Read through the Europe PMC full text. Saw segments, not wire beads."),
    _r("granite (coarse diamond lapping, PROXY for polishing)", "preston_coefficient", 1.79e-6, 1.96e-6, "mm2/N",
       "https://doi.org/10.3103/S1063457608060087", "paper", "page read",
       "Sidorko et al. 2008, Tables 4 and 6: k_p = Q/(F v), F 600 N at 60 kPa on a 100 x 100 mm plate, v = 1000 m in 55 min = "
       "303 mm/s, AS400 1000/800 diamond. A coarse lapping test, not resin-pad polishing per grit."),
    _r("marble (coarse diamond lapping, PROXY for polishing)", "preston_coefficient", 3.23e-5, 7.33e-5, "mm2/N",
       "https://doi.org/10.3103/S1063457608060087", "paper", "page read",
       "Sidorko et al. 2008, as the granite row; marble onyx 1.21e-4 and quartzite 4.3-6.4e-7 mm2/N in the same tables."),
    _r("stones (lapping test, Sidorko 2008)", "polish_pressure", 60, 60, "kPa",
       "https://doi.org/10.3103/S1063457608060087", "paper", "page read",
       "600 N on a 100 x 100 mm plate = 60 kPa; the lapping test of the Preston rows."),
]

# Joint stiffness rows by lithocam robot key. Robots match their row EXACTLY: a substring match let a row that named
# other robots in its text ("not a ... KR120") supply a 14 kg cobot's stiffness to the KR120. A robot without an
# entry has no sourced stiffness and the deflection check says so.
ROBOT_STIFFNESS_ROW = {
    "kr120": "KUKA KR210 R2700 (PROXY for a KR120-class arm)",
    "kr480_r3330_mt": "KUKA KR 500-3 (PROXY for the KR 480 / KR 500 MT cells)",
    "kr500_r2830_mt": "KUKA KR 500-3 (PROXY for the KR 480 / KR 500 MT cells)",
    "irb6700": "ABB IRB 6660 (PROXY for an IRB 6700-class arm)",
}


def robot_stiffness_rows(robot_name: str) -> List[Dict]:
    """joint_stiffness rows for a robot: the row of its lithocam key (ROBOT_STIFFNESS_ROW), else rows whose material
    equals the name exactly. Never a substring match."""
    name = robot_name.strip().lower()
    want = ROBOT_STIFFNESS_ROW.get(name, robot_name).strip().lower()
    return [r for r in TABLE if r["quantity"] == "joint_stiffness" and r["material"].lower() == want]


def lookup(material: str, quantity: Optional[str] = None) -> List[Dict]:
    """Rows whose material contains `material` (case-insensitive) and, if given, whose quantity equals `quantity`."""
    m = material.lower()
    return [r for r in TABLE if m in r["material"].lower() and (quantity is None or r["quantity"] == quantity)]
