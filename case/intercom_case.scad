// =====================================================
// ESP32-S3-ES3C28P Intercom Case v2
// Корпус для интеркома с 4 кнопками Cherry MX
// На основе STEP модели платы
// Сборка: саморезы M2.6x8mm
// =====================================================

// Параметры для настройки
$fn = 50;  // Качество окружностей

// ===== Размеры из STEP модели (плата + дисплей) =====
// Плата с дисплеем в горизонтальной ориентации
// STEP: X=-52.75..+25 Y=-34.6..+92 Z=0..17.5
// Переводим в положительные координаты и поворачиваем для горизонтальной ориентации

// Плата (без дисплея): ~50x78mm
pcb_width = 78;          // Короткая сторона платы (была Y в STEP)
pcb_length = 50;         // Длинная сторона платы (была X в STEP)
pcb_thickness = 1.6;

// Дисплей 2.8" ILI9341 (горизонтально)
// Модуль дисплея выступает за плату
display_module_width = 85;   // Ширина модуля дисплея (горизонтально)
display_module_length = 55;  // Длина модуля дисплея
display_visible_width = 58;  // Видимая область (240px, короткая сторона стала длинной)
display_visible_length = 43; // Видимая область (320px, длинная сторона стала короткой)
display_thickness = 4;       // Толщина модуля дисплея

// USB-C порт - сбоку платы
usbc_width = 9.5;
usbc_height = 3.5;

// Динамик 20x30x6.8мм (из фото)
speaker_width = 20;
speaker_length = 30;
speaker_height = 6.8;

// Микрофон - справа сверху от дисплея (интегрирован в плату)
mic_hole_d = 3;  // Отверстие под микрофон

// ===== Размеры корпуса =====
wall = 2.5;              // Толщина стенок
bottom_thickness = 2.5;  // Толщина дна
top_thickness = 3;       // Толщина крышки

// Корпус вмещает: плату + дисплей + динамик снизу + кнопки снизу
case_width = display_module_width + wall * 2 + 2;   // ~92mm
case_length = display_module_length + 45;           // Длина: дисплей + секция кнопок ~100mm
case_depth = 18;  // Глубина: плата + компоненты снизу + динамик

// Секция кнопок ниже дисплея
buttons_section = 40;  // Высота секции с кнопками

// ===== Cherry MX кнопки =====
cherry_hole = 14;        // Отверстие под Cherry MX (14x14мм)
cherry_spacing = 19.05;  // Стандартный шаг (0.75")
num_buttons = 4;

// ===== Крепёжные отверстия M2.6 =====
screw_d = 2.6;
screw_head_d = 5.5;
screw_boss_d = 6.5;
screw_inset = 6;

// ===== Позиции крепёжных отверстий =====
function screw_positions() = [
    [screw_inset, screw_inset],
    [case_width - screw_inset, screw_inset],
    [screw_inset, case_length - screw_inset],
    [case_width - screw_inset, case_length - screw_inset]
];

// ===== Модули =====

// Бобышка для самореза
module screw_boss(height) {
    difference() {
        cylinder(d=screw_boss_d, h=height);
        translate([0, 0, 2])
            cylinder(d=screw_d - 0.4, h=height);
    }
}

// Скруглённый прямоугольник
module rounded_box(w, l, h, r=3) {
    hull() {
        for (x = [r, w - r]) {
            for (y = [r, l - r]) {
                translate([x, y, 0])
                    cylinder(r=r, h=h);
            }
        }
    }
}

// Нижняя часть корпуса (основание)
module case_bottom() {
    difference() {
        union() {
            // Основной корпус
            rounded_box(case_width, case_length, case_depth);
            
            // Бобышки для саморезов
            for (pos = screw_positions()) {
                translate([pos[0], pos[1], 0])
                    screw_boss(case_depth - 3);
            }
        }
        
        // Внутренняя полость
        translate([wall, wall, bottom_thickness])
            rounded_box(case_width - wall*2, case_length - wall*2, case_depth, r=2);
        
        // USB-C порт (справа, в середине по длине дисплея)
        usbc_y = buttons_section + display_module_length/2;
        translate([case_width - wall - 1, usbc_y - usbc_width/2, bottom_thickness + 3])
            cube([wall + 2, usbc_width, usbc_height]);
        
        // Полка для платы (выступы внутри)
        // Плата лежит на высоте ~8мм от дна (под ней динамик)
        
        // Отверстия под саморезы снизу (потай)
        for (pos = screw_positions()) {
            translate([pos[0], pos[1], -1])
                cylinder(d=screw_d + 0.3, h=bottom_thickness + 2);
            // Потай
            translate([pos[0], pos[1], -0.1])
                cylinder(d1=screw_head_d + 0.5, d2=screw_d + 0.3, h=2);
        }
        
        // Отверстия для динамика (овальная решётка)
        // Динамик в центре нижней части
        speaker_x = case_width / 2;
        speaker_y = buttons_section / 2 + 5;
        
        for (i = [-2:2]) {
            for (j = [-1:1]) {
                translate([speaker_x + i*5, speaker_y + j*6, -1])
                    cylinder(d=3, h=bottom_thickness + 2);
            }
        }
    }
}

// Полка для платы внутри корпуса
module pcb_shelf() {
    shelf_height = 8;  // Высота полки (под платой динамик)
    shelf_width = 3;
    
    // Боковые полки
    translate([wall, wall + buttons_section, bottom_thickness])
        cube([shelf_width, display_module_length - 10, shelf_height]);
    translate([case_width - wall - shelf_width, wall + buttons_section, bottom_thickness])
        cube([shelf_width, display_module_length - 10, shelf_height]);
}

// Верхняя часть корпуса (крышка)
module case_top() {
    lip_height = 2;  // Выступ для фиксации
    
    difference() {
        union() {
            // Основная крышка
            rounded_box(case_width, case_length, top_thickness);
            
            // Выступ для фиксации внутри корпуса
            translate([wall + 0.3, wall + 0.3, -lip_height])
                rounded_box(case_width - wall*2 - 0.6, case_length - wall*2 - 0.6, lip_height + 0.1, r=1.5);
        }
        
        // === ВЫРЕЗ ПОД ДИСПЛЕЙ ===
        // Дисплей в верхней части корпуса (горизонтально)
        display_x = (case_width - display_visible_width) / 2;
        display_y = buttons_section + (display_module_length - display_visible_length) / 2 + 3;
        
        // Видимая область дисплея
        translate([display_x, display_y, -lip_height - 1])
            cube([display_visible_width, display_visible_length, top_thickness + lip_height + 2]);
        
        // Утопление для рамки дисплея
        translate([display_x - 3, display_y - 3, top_thickness - 1.5])
            cube([display_visible_width + 6, display_visible_length + 6, 2]);
        
        // === ОТВЕРСТИЕ ПОД МИКРОФОН ===
        // Справа сверху от дисплея
        mic_x = display_x + display_visible_width + 8;
        mic_y = display_y + display_visible_length - 5;
        translate([mic_x, mic_y, -lip_height - 1])
            cylinder(d=mic_hole_d, h=top_thickness + lip_height + 2);
        
        // === ОТВЕРСТИЯ ПОД CHERRY MX КНОПКИ ===
        // 4 кнопки в ряд в нижней секции
        // Cherry MX защёлкивается в пластину 1.5мм (допуск 1.2-1.8мм)
        cherry_plate_thickness = 1.5;  // Толщина для защёлкивания
        cherry_recess = top_thickness - cherry_plate_thickness;  // Глубина выборки
        
        buttons_total_width = (num_buttons - 1) * cherry_spacing + cherry_hole;
        buttons_start_x = (case_width - buttons_total_width) / 2 + cherry_hole/2;
        buttons_y = buttons_section / 2 + 3;
        
        for (i = [0:num_buttons-1]) {
            btn_x = buttons_start_x + i * cherry_spacing;
            
            // Квадратное отверстие 14x14 (сквозное)
            translate([btn_x - cherry_hole/2, buttons_y - cherry_hole/2, -lip_height - 1])
                cube([cherry_hole, cherry_hole, top_thickness + lip_height + 2]);
            
            // Выборка сверху для уменьшения толщины до 1.5мм
            // Размер выборки чуть больше отверстия для защёлок
            translate([btn_x - cherry_hole/2 - 1, buttons_y - cherry_hole/2 - 1, top_thickness - cherry_recess])
                cube([cherry_hole + 2, cherry_hole + 2, cherry_recess + 1]);
        }
        
        // === ОТВЕРСТИЯ ПОД САМОРЕЗЫ ===
        for (pos = screw_positions()) {
            // Сквозное отверстие
            translate([pos[0], pos[1], -lip_height - 1])
                cylinder(d=screw_d + 0.5, h=top_thickness + lip_height + 2);
            // Потай под головку
            translate([pos[0], pos[1], top_thickness - 2])
                cylinder(d1=screw_d + 0.5, d2=screw_head_d + 0.5, h=2.5);
        }
    }
}

// Динамик (для визуализации)
module speaker_model() {
    color("DimGray")
    translate([case_width/2 - speaker_width/2, buttons_section/2 + 5 - speaker_length/2, bottom_thickness + 0.5])
        cube([speaker_width, speaker_length, speaker_height]);
}

// Превью собранного корпуса
module assembled_preview() {
    color("DarkSlateGray") case_bottom();
    color("DarkSlateGray", 0.3) pcb_shelf();
    // speaker_model();  // Раскомментировать для показа динамика
    color("SlateGray", 0.7) translate([0, 0, case_depth]) case_top();
}

// ===== РЕНДЕРИНГ =====
// Раскомментируйте нужный вариант:

// Вариант 1: Превью собранного корпуса
assembled_preview();

// Вариант 2: Только нижняя часть (для экспорта STL)
// case_bottom();

// Вариант 3: Только верхняя часть (для экспорта STL)  
// case_top();

// Вариант 4: Детали для печати (разнесены)
// case_bottom();
// translate([case_width + 10, 0, top_thickness]) rotate([180, 0, 0]) case_top();

// =====================================================
// ХАРАКТЕРИСТИКИ КОРПУСА:
// =====================================================
// Внешние размеры: ~92 x 100 x 21 мм
// 
// Особенности:
// - Дисплей 2.8" в горизонтальной ориентации
// - 4 кнопки Cherry MX под дисплеем
// - Динамик 20x30 мм внутри (решётка снизу)
// - Микрофон - отверстие справа сверху от дисплея
// - USB-C порт справа
// - Крепление на 4 самореза M2.6x8
//
// ПЕЧАТЬ:
// - Слой: 0.2мм
// - Заполнение: 15-20%
// - Материал: PLA или PETG
// - Поддержки: не нужны
//
// СБОРКА:
// 1. Установить динамик в нижнюю часть
// 2. Установить плату на полки
// 3. Вставить Cherry MX кнопки в крышку
// 4. Закрыть крышку и закрутить саморезы
// =====================================================
