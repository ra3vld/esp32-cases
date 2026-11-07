import {fileURLToPath} from 'node:url';
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE||'playwright');
import fs from 'node:fs';
import crypto from 'node:crypto';
const root=fileURLToPath(new URL('../output',import.meta.url));
const html='capsule-3d.html';
const browser=await chromium.launch({executablePath:'/usr/bin/chromium',headless:true,args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:1300,height:1000}});
const errors=[];page.on('pageerror',e=>errors.push(e.message));
await page.goto('file://'+root+'/'+html);
await page.waitForFunction(()=>document.querySelector('canvas').dataset.ready==='true');
const modes=[];
for(const mode of ['assembly','service','shell','exploded','bottom']){
 await page.locator(`[data-view="${mode}"]`).click();
 await page.waitForFunction(m=>window.__cadViewer.mode===m,mode);
 modes.push(await page.evaluate(()=>window.__cadViewer));
 if(mode==='shell')await page.screenshot({path:root+'/viewer-shell-preview.png',fullPage:true});
}
await page.locator('[data-view="assembly"]').click();
await page.waitForFunction(()=>window.__cadViewer.mode==='assembly');
await page.screenshot({path:root+'/viewer-preview.png',fullPage:true});
for(const id of ['wire','cut','cups','hardware']){await page.locator('#'+id).click();await page.evaluate(()=>new Promise(requestAnimationFrame));if(await page.evaluate(()=>window.__cadViewer.glError))errors.push('GL '+id);}
await page.setViewportSize({width:390,height:844});
await page.evaluate(()=>new Promise(requestAnimationFrame));
const mobileOverflow=await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth);
const result={status:errors.length===0&&!mobileOverflow&&modes.every(m=>m.glError===0)&&modes.find(m=>m.mode==='shell').visibleMeshes===1?'PASS':'FAIL',modes,webglErrors:errors,mobileOverflow,html_sha256:crypto.createHash('sha256').update(fs.readFileSync(root+'/'+html)).digest('hex')};
fs.writeFileSync(root+'/viewer-qa.json',JSON.stringify(result,null,2)+'\n');
console.log(JSON.stringify(result));await browser.close();
if(result.status!=='PASS')process.exit(1);
