// =====================================================
// ESP32-S3-ES3C28P Intercom Case
// Корпус для интеркома с 4 кнопками Cherry MX
// Сборка: саморезы M2.6x8mm
// =====================================================

// Параметры для настройки
$fn = 50;  // Качество окружностей

// ===== Размеры платы ESP32-S3-ES3C28P =====
pcb_width = 78;         // Ширина платы (X)
pcb_height = 52;        // Высота платы (Y) 
pcb_thickness = 1.6;    // Толщина платы

// ===== Размеры экрана 2.8" =====
screen_width = 43;      // Ширина видимой области экрана (Active Area)
screen_height = 58;     // Высота видимой области экрана
screen_bezel = 2;       // Рамка вокруг экрана

// ===== Размеры корпуса =====
wall = 2.5;             // Толщина стенок
bottom_thickness = 2.5; // Толщина дна
top_lip = 2;            // Выступ крышки для фиксации

case_width = pcb_width + wall * 2 + 1;    // ~84mm
case_height = pcb_height + wall * 2 + 1;  // ~58mm
case_depth = 22;        // Глубина корпуса

// ===== Секция кнопок (расширение корпуса вниз) =====
buttons_section_height = 30;  // Высота секции с кнопками
total_height = case_height + buttons_section_height;

// ===== Cherry MX кнопки =====
cherry_hole = 14;       // Отверстие под Cherry MX (14x14мм стандарт)
cherry_spacing = 19.05; // Стандартный шаг между кнопками (0.75" = 19.05mm)
num_buttons = 4;
buttons_y_offset = 15;  // Центр кнопок от нижнего края

// ===== USB-C порт =====
usbc_width = 10;        // Ширина отверстия под USB-C
usbc_height = 4;        // Высота отверстия
usbc_z_offset = 4;      // Высота от дна корпуса

// ===== Динамик/Микрофон =====
speaker_holes = true;   // Отверстия для звука

// ===== Крепёжные отверстия M2.6 =====
screw_d = 2.6;          // Диаметр самореза
screw_head_d = 5.5;     // Диаметр головки
screw_boss_d = 6.5;     // Диаметр бобышки
screw_inset = 5;        // Отступ от края

// ===== Позиции крепёжных отверстий =====
function screw_positions() = [
    [screw_inset, screw_inset],
    [case_width - screw_inset, screw_inset],
    [screw_inset, total_height - screw_inset],
    [case_width - screw_inset, total_height - screw_inset]
];

// ===== Модули =====

// Бобышка для самореза
module screw_boss(height) {
    difference() {
        cylinder(d=screw_boss_d, h=height);
        translate([0, 0, 2])
            cylinder(d=screw_d - 0.4, h=height);  // Отверстие под саморез
    }
}

// Нижняя часть корпуса (основание)
module case_bottom() {
    difference() {
        union() {
            // Основной корпус с скруглёнными углами
            hull() {
                for (x = [3, case_width - 3]) {
                    for (y = [3, total_height - 3]) {
                        translate([x, y, 0])
                            cylinder(r=3, h=case_depth);
                    }
                }
            }
            
            // Бобышки для саморезов
            for (pos = screw_positions()) {
                translate([pos[0], pos[1], 0])
                    screw_boss(case_depth - 3);
            }
        }
        
        // Внутренняя полость
        translate([wall, wall, bottom_thickness])
            hull() {
                for (x = [2, case_width - wall*2 - 2]) {
                    for (y = [2, total_height - wall*2 - 2]) {
                        translate([x, y, 0])
                            cylinder(r=2, h=case_depth);
                    }
                }
            }
        
        // USB-C порт (справа, в секции платы)
        translate([case_width - wall - 1, 
                   buttons_section_height + case_height/2 - usbc_width/2, 
                   bottom_thickness + usbc_z_offset])
            cube([wall + 2, usbc_width, usbc_height]);
        
        // Отверстия под саморезы снизу
        for (pos = screw_positions()) {
            translate([pos[0], pos[1], -1])
                cylinder(d=screw_d + 0.3, h=bottom_thickness + 2);
        }
        
        // Вентиляционные отверстия снизу (под платой)
        if (speaker_holes) {
            // Отверстия для динамика/микрофона
            for (i = [-2:2]) {
                for (j = [-2:2]) {
                    if (abs(i) + abs(j) <= 3) {
                        translate([case_width/2 + i*4, 
                                   buttons_section_height + case_height/2 + j*4, 
                                   -1])
                            cylinder(d=2.5, h=bottom_thickness + 2);
                    }
                }
            }
        }
    }
}

// Верхняя часть корпуса (крышка с экраном и кнопками)
module case_top() {
    top_thickness = 3;
    
    difference() {
        union() {
            // Основная крышка с скруглёнными углами
            hull() {
                for (x = [3, case_width - 3]) {
                    for (y = [3, total_height - 3]) {
                        translate([x, y, 0])
                            cylinder(r=3, h=top_thickness);
                    }
                }
            }
            
            // Выступ для фиксации внутри корпуса
            translate([wall + 0.3, wall + 0.3, -top_lip])
                hull() {
                    for (x = [1, case_width - wall*2 - 0.6 - 1]) {
                        for (y = [1, total_height - wall*2 - 0.6 - 1]) {
                            translate([x, y, 0])
                                cylinder(r=1, h=top_lip + 0.1);
                        }
                    }
                }
        }
        
        // Отверстие под экран (с рамкой)
        screen_x = (case_width - screen_width - screen_bezel*2) / 2;
        screen_y = buttons_section_height + (case_height - screen_height - screen_bezel*2) / 2;
        
        // Видимая область экрана
        translate([screen_x + screen_bezel, screen_y + screen_bezel, -top_lip - 1])
            cube([screen_width, screen_height, top_thickness + top_lip + 2]);
        
        // Утопление для стекла/модуля экрана
        translate([screen_x, screen_y, top_thickness - 1])
            cube([screen_width + screen_bezel*2, screen_height + screen_bezel*2, 2]);
        
        // Отверстия под Cherry MX кнопки (4 штуки в ряд)
        buttons_total_width = (num_buttons - 1) * cherry_spacing + cherry_hole;
        buttons_start_x = (case_width - buttons_total_width) / 2 + cherry_hole/2;
        
        for (i = [0:num_buttons-1]) {
            btn_x = buttons_start_x + i * cherry_spacing;
            btn_y = buttons_y_offset;
            
            // Квадратное отверстие 14x14 для Cherry MX
            translate([btn_x - cherry_hole/2, btn_y - cherry_hole/2, -top_lip - 1])
                cube([cherry_hole, cherry_hole, top_thickness + top_lip + 2]);
        }
        
        // Отверстия под саморезы (с потаем)
        for (pos = screw_positions()) {
            // Сквозное отверстие
            translate([pos[0], pos[1], -top_lip - 1])
                cylinder(d=screw_d + 0.5, h=top_thickness + top_lip + 2);
            // Потай под головку
            translate([pos[0], pos[1], top_thickness - 2])
                cylinder(d1=screw_d + 0.5, d2=screw_head_d + 0.5, h=2.5);
        }
    }
}

// Превью собранного корпуса
module assembled_preview() {
    color("DarkSlateGray") case_bottom();
    color("SlateGray", 0.8) translate([0, 0, case_depth]) case_top();
}

// ===== Рендеринг =====
// Раскомментируйте нужный вариант:

// Вариант 1: Превью собранного корпуса
assembled_preview();

// Вариант 2: Детали разнесены для печати
// case_bottom();
// translate([case_width + 10, 0, top_thickness]) rotate([180, 0, 0]) case_top();

// Вариант 3: Только нижняя часть (для экспорта STL)
// case_bottom();

// Вариант 4: Только верхняя часть (для экспорта STL)
// case_top();

// =====================================================
// ИНСТРУКЦИИ:
// =====================================================
// 1. Установите OpenSCAD: https://openscad.org/downloads.html
// 2. Откройте этот файл
// 3. Для экспорта в STL:
//    a) Закомментируйте assembled_preview()
//    b) Раскомментируйте case_bottom() или case_top()
//    c) Нажмите F6 (Render)
//    d) File -> Export -> Export as STL
//
// Размеры корпуса: ~84 x 88 x 25 мм
//
// Печать:
// - Сопло: 0.4мм
// - Слой: 0.2мм  
// - Заполнение: 15-20%
// - Материал: PLA или PETG
// - Поддержки: не нужны
//
// Сборка:
// - 4x саморезы M2.6x8mm
// - Плата ESP32-S3-ES3C28P
// - 4x переключатели Cherry MX
// =====================================================
