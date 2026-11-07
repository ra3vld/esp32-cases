"""A4 landscape vector manufacturing drawings, derived from the exported BREP."""
from io import BytesIO
import json
import math
from pathlib import Path
import cadquery as cq
from reportlab.pdfgen import canvas
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4,landscape
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import Paragraph
from reportlab.lib.styles import ParagraphStyle
from reportlab.graphics import renderPDF
from svglib.svglib import svg2rlg
import build as b

ROOT=b.ROOT;OUT=b.OUT;P=b.P
W,H=landscape(A4);MM=72/25.4
FONT='/usr/share/fonts/truetype/dejavu/'
pdfmetrics.registerFont(TTFont('DV',FONT+'DejaVuSans.ttf'))
pdfmetrics.registerFont(TTFont('DVB',FONT+'DejaVuSans-Bold.ttf'))
INK=colors.HexColor('#243746');BLUE=colors.HexColor('#276889');GRAY=colors.HexColor('#637785')
STYLE=ParagraphStyle('notes',fontName='DV',fontSize=9,leading=13,textColor=INK)

def txt(c,x,y,s,size=9,bold=False,color=INK):
    c.setFillColor(color);c.setFont('DVB' if bold else 'DV',size);c.drawString(x,y,str(s))

def para(c,x,y,width,s,size=9):
    style=ParagraphStyle('p',parent=STYLE,fontSize=size,leading=size*1.45)
    p=Paragraph(s,style);_,height=p.wrap(width,1000)
    p.drawOn(c,x,y-height);return y-height

def head(c,title,index):
    c.setStrokeColor(INK);c.setLineWidth(.7);c.rect(20,20,W-40,H-40)
    txt(c,36,H-48,title,17,True)
    txt(c,36,H-67,'REMOTE LDGCONTROL / ESP32-3248S035C / КАПСУЛА 0.1',9,color=GRAY)
    c.line(20,58,W-20,58)
    txt(c,32,40,'мм / Размеры по CAD / 07.11.2026 / PETG, сопло 0,4 мм',8)
    txt(c,440,40,'ПЕРЕД ПОЛНОЙ ПЕЧАТЬЮ НАПЕЧАТАТЬ ПРОБНИКИ',8,True)
    txt(c,W-72,40,f'{index}/10',8)

def arrow(c,x,y,dx,dy):
    n=math.hypot(dx,dy);dx/=n;dy/=n
    c.line(x,y,x+dx*4-dy*1.7,y+dy*4+dx*1.7)
    c.line(x,y,x+dx*4+dy*1.7,y+dy*4-dx*1.7)

def dim(c,x1,y1,x2,y2,label):
    c.setStrokeColor(BLUE);c.setLineWidth(.55);c.line(x1,y1,x2,y2)
    arrow(c,x1,y1,x2-x1,y2-y1);arrow(c,x2,y2,x1-x2,y1-y2)
    txt(c,(x1+x2)/2+4,(y1+y2)/2+6,label,9,color=BLUE)

def view(c,shape,x,y,w,h,direction=(0,0,1),name='view'):
    # Explicit camera roll: x is horizontal and z is vertical in elevations.
    if direction==(0,-1,0):shape=shape.rotate((0,0,0),(1,0,0),-90)
    elif direction==(0,1,0):shape=shape.rotate((0,0,0),(1,0,0),90).rotate((0,0,0),(0,0,1),180)
    elif direction==(1,0,0):shape=shape.rotate((0,0,0),(1,1,1),-120)
    elif direction==(0,0,-1):shape=shape.mirror('XY')
    bb=shape.BoundingBox();scale=min((w-8)/bb.xlen,(h-8)/bb.ylen)
    options={'width':bb.xlen*scale+8,'height':None,'marginLeft':4,'marginTop':4,'projectionDir':(0,0,1),
             'showAxes':False,'showHidden':False,'strokeWidth':.35,'strokeColor':(36,55,70)}
    svg=cq.exporters.getSVG(shape,options)
    folder=OUT/'drawing_views';folder.mkdir(exist_ok=True)
    (folder/(name+'.svg')).write_text(svg)
    drawing=svg2rlg(BytesIO(svg.encode()))
    xx=x+(w-drawing.width)/2;yy=y+(h-drawing.height)/2
    renderPDF.draw(drawing,c,xx,yy)
    return (xx+4,yy+4,xx+drawing.width-4,yy+drawing.height-4)

def local(shape):
    return shape.translate(tuple(-v for v in b.ORIGIN)).rotate((0,0,0),(1,0,0),P['tilt_back_deg']-90).mirror('XY')

def table(c,x,y,width,rows,col=0.46):
    for left,right in rows:
        heights=[]
        for value,ww in [(left,width*col-8),(right,width*(1-col)-8)]:
            pp=Paragraph(str(value),ParagraphStyle('cell',parent=STYLE,fontSize=8.5,leading=8.5*1.45))
            heights.append(pp.wrap(ww,1000)[1])
        height=max(24,max(heights)+12)
        c.setStrokeColor(colors.HexColor('#d8e0e5'));c.setLineWidth(.4);c.line(x,y,x+width,y)
        para(c,x+4,y-5,width*col-8,str(left),8.5)
        para(c,x+width*col+4,y-5,width*(1-col)-8,str(right),8.5)
        y-=height
    return y

def run():
    parts={f.stem:cq.importers.importStep(str(f)).val() for f in sorted((OUT/'step').glob('*.step'))}
    stock=[cq.importers.importStep(str(OUT/'reference_step'/f'{name}.step')).val()
           for name in ('ref_pcb','ref_glass','ref_screen')]
    production=cq.Compound.makeCompound([v for k,v in parts.items() if k[:2] in ('01','02','03','04','05','06')]+stock)
    dims=json.loads((OUT/'dimensions.json').read_text())['enclosure_xyz_mm']
    size=' × '.join(f'{n:.1f}'.replace('.',',') for n in dims)
    c=canvas.Canvas(str(OUT/'capsule-v01-drawings.pdf'),pagesize=(W,H))
    c.setTitle('Remote LDGControl - Капсула 0.1 - размерные чертежи')
    c.setAuthor('Remote-LDGControl / parametric CadQuery build')

    head(c,'01 / Сборочный чертёж',1)
    a=view(c,production,45,295,355,215,(0,-1,0),'assembly-front')
    d=view(c,production,440,295,320,215,(1,0,0),'assembly-side')
    dim(c,a[0],a[1]-13,a[2],a[1]-13,f'{dims[0]:.1f}');dim(c,d[0],d[1]-13,d[2],d[1]-13,f'{dims[1]:.1f}')
    dim(c,d[2]+15,d[1],d[2]+15,d[3],f'{dims[2]:.1f}')
    table(c,45,238,355,[('Дисплей','480 × 320, горизонтально'),('Наклон от вертикали','15° назад / 75° к столу'),('Габарит корпуса',size+' без присосок'),('Присоски','4 × Ø20; посадка Ø6')])
    para(c,440,235,340,'Система координат сборки: X - вправо при взгляде на экран, Y - назад, Z - вверх от низа подошвы. Наклон платы и рамки задаётся единым преобразованием.',9)
    para(c,440,165,340,'Плата и интерфейс показаны габаритными моделями. Посадочные размеры приведены в parameters.json; для примерки платы использовать пробник 08.',9)
    c.showPage()

    head(c,'02 / Разнесённая сборка и обслуживание',2)
    c.drawImage(str(OUT/'cad-construction.png'),55,105,width=730,height=420,preserveAspectRatio=True,anchor='c')
    txt(c,48,80,'Плата крепится к рамке; кассета и зажим - к подошве. RESET/BOOT: отверстия Ø2 без трубок.',8.5)
    c.showPage()

    head(c,'03 / Передняя рамка и крепление платы',3)
    a=view(c,local(parts['01_front_frame']),40,235,390,285,(0,0,-1),'frame-front')
    frame_bounds=b.exact_bounds(local(parts['01_front_frame']))
    dim(c,a[0],a[1]-13,a[2],a[1]-13,f'{frame_bounds.xlen:.1f}');dim(c,a[2]+13,a[1],a[2]+13,a[3],f'{frame_bounds.ylen:.1f}')
    table(c,465,515,325,[('Окно экрана','76,5 × 52,0; R1,5; центр U = +2'),('Лицевая стенка','2,4; в кармане стекла 1,4'),('Опоры PCB','4 × Ø8; вылет 3,8; PCB на 1 мм вперёд'),('Крепление PCB','94,5 × 47,9; M3 × 6 изнутри'),('Крепление рамки','U = ±54; V = ±21'),('Крепёж рамки','4 × M3 × 10 изнутри; лицевая стенка глухая'),('Вставки в рамке','8 × M3 / Ø4,6 × 4,0'),('Инструмент рамки','Шестигранник 2 мм с шаровым концом; наклон к центру до 10°')])
    table(c,45,178,745,[('Замкнутый бортик','1,6 × 3; по полному периметру; боковой и торцевой зазоры 0,3'),('Оптические отверстия U;V','LED Ø1,5: (47,1; 15,2); LDR Ø2: (47,1; -14,4)'),('Глубина и примерка','Стекло углублено на 1,7; зазор кармана 0,3; проверить пробником 08')],.34)
    c.showPage()

    head(c,'04 / Основной корпус и силовые опоры',4)
    view(c,parts['02_body'],45,250,355,265,(0,1,0),'body-rear')
    view(c,parts['02_body'],435,250,350,265,(1,0,0),'body-side')
    table(c,45,235,740,[('Оболочка и углы','Стенка 2,4; R48; передний переход R2,4; силовые опоры толще'),('Задние стойки M3','Ø14; нижние X = ±47, Z = 28; верхние X = ±40, Z = 57'),('Основания задних стоек','Высота 14; Y = 56…68,4; широкое цельное соединение со стенкой'),('Пилоты термовставок','Ø4,2, глубина 4,4 под вставку Ø4,6 × 4,0'),('Micro-USB','Открытая боковая выборка под штатный разъём и штекер 23 × 12 × 6,8'),('Дно и печать','Перемычки по контуру сохранены; центр дна свободен. Поддержки под стойками')],.3)
    c.showPage()

    head(c,'05 / Задняя крышка и отверстия кнопок',5)
    view(c,parts['03_rear_cover'],40,235,385,285,(0,1,0),'rear-outside')
    view(c,parts['03_rear_cover'],450,300,335,220,(1,0,0),'rear-side')
    table(c,455,278,330,[('Крепление','4 × M3 × 12; головки в колодцах'),('Отверстия скрепки','2 × Ø2; скрепка Ø1'),('RESET и BOOT','Без выступающих направляющих трубок'),('Ось отверстий','Перпендикулярно PCB; наклон 15°'),('Кабельный выход','Ø5,6 под кабель Ø5'),('Центр выхода X; Z','-32; 13')])
    para(c,45,180,365,'Кнопки: (U;V) RESET = (33,2;-21), BOOT = (29,3;-21). Отверстия Ø2 проходят только через стенку крышки; скрепку вести вдоль оси отверстия. Сверить оси с кнопками своей платы до полной печати.',9)
    para(c,45,112,365,'Печать задней центральной площадкой вниз. Поддержки локально под краями; отверстия кнопок Ø2 оставить свободными.',9)
    c.showPage()

    head(c,'06 / Подошва и присоски',6)
    a=view(c,parts['04_sole'],40,205,430,310,(0,0,1),'sole-top')
    sole_bounds=b.exact_bounds(parts['04_sole'])
    dim(c,a[0],a[1]-13,a[2],a[1]-13,f'{sole_bounds.xlen:.1f}');dim(c,a[2]+13,a[1],a[2]+13,a[3],f'{sole_bounds.ylen:.1f}')
    table(c,500,515,285,[('Толщина подошвы','3,2'),('Передние присоски','X = ±44; Y = 20'),('Задние присоски','X = ±54; Y = 62'),('Посадки','4 × Ø6 насквозь'),('Толщина под шейкой','2,0'),('Полость для головки','Ø12,8 выше Z = 2'),('Винты подошвы','4 × M3 × 6'),('Центры винтов','X = ±53; Y = 28 и 50'),('Бортик','Замкнутый 134 × 67,7; R25; центр Y = 42,15')])
    para(c,45,150,420,'Присоски покупные, Ø20 мм, хвостик Ø6 мм. Подобрать посадку на пробнике 07. Крепёжные винты утоплены в подошву.',9)
    para(c,45,92,420,'Нижние зенковки Ø6,2 глубиной 1,7 под низкую головку ISO 7380-1. Шаг резьбы M3 - 0,5.',9)
    c.showPage()

    head(c,'07 / Кассета интерфейса и кабельный прижим',7)
    view(c,parts['05_interface_cassette'],45,260,360,250,(0,0,1),'cassette-top')
    view(c,parts['06_cable_clamp'],460,315,315,180,(0,0,1),'clamp-top')
    table(c,45,220,355,[('Кассета с бортиком','44,2 × 29,2; дно 2,4'),('Посадочный карман','41 × 26; модуль 40 × 25'),('Полный модуль','40 × 25 × 25; низ Z = 8'),('Центр модуля X; Y','6; 46'),('Крепление кассеты','2 × M3 × 6'),('Стяжки','2 × ширина 2,5; длина от 150')])
    table(c,455,275,330,[('Прижим','30 × 10; толщина 6,5; штифты Ø2,4 × 2'),('Кабель','Ø5; ось Y'),('Центр кабеля X; Z','-32; 13'),('Центр зажима Y','60,5'),('Крепление','2 × M3 × 8, шаг 20; два направляющих штифта'),('Зазор половин','0,3 до затяжки')])
    c.showPage()

    head(c,'08 / Разрез стыка A-A и посадки M3',8)
    # Enlarged exact nominal cross-section of one tongue flank.
    scale=32;ox=185;oy=145
    def rect(x,z,w,h,fill):
        c.setFillColor(fill);c.setStrokeColor(INK);c.setLineWidth(.7);c.rect(ox+x*scale,oy+z*scale,w*scale,h*scale,fill=1)
    rect(-3,0,7.6,2.4,colors.HexColor('#cbd7df'))
    path=c.beginPath();path.moveTo(ox,oy+2.4*scale);path.lineTo(ox+1.6*scale,oy+2.4*scale);path.lineTo(ox+1.6*scale,oy+5.2*scale);path.lineTo(ox+1.4*scale,oy+5.4*scale);path.lineTo(ox+.2*scale,oy+5.4*scale);path.lineTo(ox,oy+5.2*scale);path.close();c.drawPath(path,fill=1,stroke=1)
    rect(-3,2.7,2.7,3.0,colors.HexColor('#e7ddc8'));rect(-3,5.7,4.9,2.4,colors.HexColor('#e7ddc8'))
    dim(c,ox,oy+2.1*scale,ox+1.6*scale,oy+2.1*scale,'1,6')
    dim(c,ox-3.5*scale,oy+2.4*scale,ox-3.5*scale,oy+5.4*scale,'3,0')
    dim(c,ox+5.2*scale,oy+2.4*scale,ox+5.2*scale,oy+5.7*scale,'3,3 от датума')
    txt(c,70,443,'A-A / увеличенный разрез стыка',11,True)
    table(c,455,485,330,[('Боковой зазор','0,3 до направляющей стенки'),('Бортик крышки','1,6 × 3; непрерывный замкнутый контур'),('Зазор торца бортика','Не менее 0,3; в USB-окне упор открыт'),('Разгрузка наружных торцов','0,3'),('Заходная фаска','0,2 × 45°'),('Сборочный датум','Опорные площадки винтов')])
    para(c,455,280,330,'Бортик выступает на крышке и входит внутрь основной детали. Приёмная ступень направляет его наружную сторону по всему периметру. От сборочного датума до упора 3,3 мм; заход 2,7 мм, запас за торцом 0,3 мм. Внутренняя сторона выходит в полость.',9)
    para(c,455,170,330,'Вставки M3: наружный Ø4,6 × 4,0. Начальный пилот Ø4,2 × 4,4; диаметр уточняется пробником. Радиальный натяг 0,2 предусмотрен только под термовставку.',9)
    c.showPage()

    head(c,'09 / Пробники перед полной печатью',9)
    view(c,parts['07_fit_coupon'],40,255,390,250,(0,0,1),'coupon-fit')
    view(c,local(parts['08_board_coupon']),450,255,340,250,(0,0,-1),'coupon-board')
    para(c,45,225,370,'07: сверху слева направо отверстия 5,8 / 6,0 / 6,2 мм и пилоты вставок 4,0 / 4,2 / 4,4 мм. Два нижних элемента отделить по перемычкам 0,6 мм; правый перевернуть и надеть на левый.',9)
    para(c,45,133,370,'Проверить посадку присоски и свободное соединение стыка без силового вдавливания. Общий масштаб модели не менять: корректировать только соответствующий параметр.',9)
    para(c,455,225,330,'08: примерить плату, четыре крепления, стекло, USB-штекер и расположение световых отверстий. Проверить зазор стекла и отсутствие изгиба PCB. Не вплавлять вставки рядом с установленной платой.',9)
    para(c,455,115,330,'Контроль осей кнопок: временно установить заднюю крышку либо сверить отдельной примеркой до печати полного комплекта.',9)
    c.showPage()

    head(c,'10 / Комплектация, печать и приёмка',10)
    table(c,45,510,350,[('M3 × 6, ISO 7380-1','10 шт.'),('M3 × 8, ISO 7380-1','2 шт. / прижим'),('M3 × 10, ISO 7380-1','4 шт. / рамка'),('M3 × 12, ISO 7380-1','4 шт.'),('Термовставка M3 Ø4,6 × 4','20 шт.'),('Присоска Ø20 под Ø6','4 шт.'),('Нейлоновая стяжка 2,5 × 150','2 шт.'),('Кабель Ø5','1 шт.'),('Печатные детали','6 рабочих + 2 пробника')])
    para(c,445,510,340,'Профиль: PETG, сопло 0,4, слой 0,2; 4 периметра, 5 верхних/нижних слоёв, заполнение 25%. У стоек и корней - локально 6 периметров и 50% заполнения. Температуры по своему пластику.',9)
    para(c,445,407,340,'Сборка: пробники → печать → вставки → плата к рамке → рамка к корпусу → интерфейс и зажим к подошве → задняя крышка → проводка → подошва и присоски. Для разборки освободить зажим и проводку, снять подошву, затем заднюю крышку.',9)
    para(c,445,294,340,'После сборки: подключить USB, нажать RESET/BOOT скрепкой, проверить касания по углам экрана и датчик при включённом LED. Закрепить присоски на чистой гладкой поверхности.',9)
    para(c,45,250,350,'Основа размеров: Jingcai, ESP32-3248S035 Specifications-EN, страницы 4 и 6. Посадочные размеры - parameters.json. Перед полной печатью примерить свою плату и USB-штекер на пробнике 08.',9)
    para(c,45,172,350,'Печатать файлы output/print_stl в миллиметрах, масштаб 100%. STEP и сборочные STL имеют координаты сборки. Подробные ориентации и последовательность приведены в README.md.',9)
    c.showPage();c.save()
    print('10 vector drawing sheets exported')

if __name__=='__main__':run()
