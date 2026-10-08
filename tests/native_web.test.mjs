// Optional real-browser acceptance. Start geckodriver and the native servers,
// then set WEBDRIVER_URL, NAV_URL and DUEL_URL. No npm packages are required.
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {mkdir, writeFile} from 'node:fs/promises';

const webdriver = process.env.WEBDRIVER_URL;
const output = process.env.RM_TEST_OUTPUT || 'output/migration';
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
async function session(url, run) {
  async function request(path, body, method = body === undefined ? 'GET' : 'POST') {
    const response = await fetch(webdriver + path, {
      method, headers: {'Content-Type': 'application/json'},
      body: body === undefined ? undefined : JSON.stringify(body), signal: AbortSignal.timeout(40000),
    });
    const result = (await response.json()).value;
    if (!response.ok) throw Error(result?.message || `WebDriver HTTP ${response.status}`);
    return result;
  }
  const id = (await request('/session', {capabilities: {alwaysMatch: {
    browserName: 'firefox', 'moz:firefoxOptions': {args: ['-headless']},
  }}})).sessionId;
  const prefix = '/session/' + id;
  const js = script => request(prefix + '/execute/sync', {script, args: []});
  const wait = async script => {
    for (let i=0; i<150; i++) {if (await js(script)) return; await delay(200);}
    throw Error('UI condition timed out: ' + script);
  };
  const click = async selector => {
    await wait(`return !document.querySelector(${JSON.stringify(selector)}).disabled`);
    const element = await request(prefix + '/element', {using: 'css selector', value: selector});
    await request(prefix + '/element/' + element['element-6066-11e4-a52e-4f735466cecf'] + '/click', {});
  };
  const screenshot = async name => {
    await mkdir(output, {recursive: true});
    await writeFile(`${output}/${name}.png`, Buffer.from(await request(prefix + '/screenshot'), 'base64'));
  };
  const waitState = async predicate => {
    for (let i=0; i<150; i++) {
      const state = await (await fetch(new URL('/api/state', url), {signal: AbortSignal.timeout(3000)})).json();
      if (predicate(state)) return state;
      await delay(200);
    }
    throw Error('Native state condition timed out: ' + predicate);
  };
  try {
    await request(prefix + '/window/rect', {width: 1400, height: 1100});
    await request(prefix + '/url', {url});
    await run({request: (path, body) => request(prefix + path, body), js, wait, waitState, click, screenshot});
  } finally {await request(prefix, undefined, 'DELETE');}
}

test('native duel browser controls and rendered image', {skip: !webdriver || !process.env.DUEL_URL, timeout: 120000}, async () => {
  await session(process.env.DUEL_URL, async ({wait, waitState, click, js, screenshot}) => {
    await wait("return document.querySelector('#connection').textContent.startsWith('已连接')");
    await wait("return document.querySelector('#scene').naturalWidth === 800 && document.querySelector('#camera').naturalWidth === 800");
    await click('#reset');
    await waitState(s => s.time < 2 && !s.paused);
    await click('#pause');
    await waitState(s => s.paused);
    await wait("return document.querySelector('#pause').textContent === '继续'");
    await click('#pause');
    await waitState(s => !s.paused);
    if (!await js("return document.querySelector('#autoFire').checked")) await click('#autoFire');
    await waitState(s => s.settings.auto_fire && s.robots[0].shots > 0);
    await click('#followButton');
    await waitState(s => s.settings.view === 'follow');
    await click('#centerAim');
    await waitState(s => !s.settings.auto_aim && Math.abs(s.robots[0].gimbal[0]) < 0.1);
    await screenshot('native-duel-browser');
    assert.equal(await js("return document.querySelector('#connectionError').style.display"), 'none');
    await click('#autoAim');
  });
});

test('native navigation browser map, goal, yaw and pause', {skip: !webdriver || !process.env.NAV_URL, timeout: 120000}, async () => {
  await session(process.env.NAV_URL, async ({wait, click, request, js, screenshot}) => {
    await wait("return document.querySelector('#connectionText').textContent === '状态已连接'");
    await wait("return document.querySelector('#frame').naturalWidth > 0 && document.querySelectorAll('#presets button').length > 0");
    await click('#reset');
    await wait("return document.querySelector('#yawValue').textContent === '0°/s' && !document.querySelector('#yawRate').disabled");
    const element = await request('/element', {using: 'css selector', value: '#yawRate'});
    await request('/element/' + element['element-6066-11e4-a52e-4f735466cecf'] + '/value', {text: '\uE014\uE004'});
    await wait("return document.querySelector('#yawValue').textContent === '5°/s'");
    await click('#presets button');
    await wait("return document.querySelector('#goalName').textContent === '当前导航目标'");
    await click('#yawStop');
    await wait("return document.querySelector('#yawValue').textContent === '0°/s'");
    await click('#pause');
    await wait("return document.querySelector('#pause').textContent === '继续导航'");
    await screenshot('native-navigation-browser');
    await click('#reset');
    await wait("return document.querySelector('#goalName').textContent === '尚未设置目的地'");
    await request('/window/rect', {width: 860, height: 1100});
    assert.equal(await js('return document.documentElement.scrollWidth <= window.innerWidth'), true);
  });
});
