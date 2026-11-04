// Unranked demonstration driver linked to the unchanged Arena library.
// Only explicitly recorded referee/HP/ammunition stimuli may alter state.
use rmuc_sim::Engine;
use serde_json::{json,Value};
use std::io::{self,BufRead,Write};

fn apply(e:&mut Engine, event:&Value)->Result<(),String>{
    let now=e.time();
    match event["kind"].as_str().ok_or("missing demo event kind")? {
        "all_outposts_red"=>{
            for side in ["red","blue"] {
                let p=e.positions.get_mut(&format!("{side}_outpost")).ok_or("outpost position missing")?;
                p.controller="red".into();p.control="full".into();p.progress=0.;
                let o=e.outposts.get_mut(side).unwrap();o.owner=Some("red".into());
                if o.hp<=0. {o.hp=500.;o.max_hp=500.;}
            }
        },
        "resources"=>e.teams.get_mut("red").unwrap().resources=event["amount"].as_f64().ok_or("amount")?,
        "hp_ratio"=>{let r=e.robots.iter_mut().find(|r|r.id=="red-7").unwrap();
            r.hp=r.max_hp*event["ratio"].as_f64().ok_or("ratio")?;r.last_damage_time=now;},
        "ammo"=>e.robots.iter_mut().find(|r|r.id=="red-7").unwrap().ammo=event["amount"].as_u64().ok_or("amount")? as u32,
        "destroy"=>{let r=e.robots.iter_mut().find(|r|r.id=="red-7").unwrap();
            r.hp=0.;r.alive=false;r.death_time=now;r.last_damage_time=now;r.weak_until=None;
            r.heat=0.;r.pending_heal=None;r.respawn_progress=0.;
            r.respawn_required=(10.+now/10.+20.*r.instant_revives as f64).round();},
        _=>return Err("unsupported demo event".into()),
    }
    Ok(())
}

fn main(){
    let mut engine:Option<Engine>=None;let mut schedule:Vec<Value>=vec![];let mut cursor=0;
    let stdin=io::stdin();let mut out=io::BufWriter::new(io::stdout());
    for line in stdin.lock().lines(){
        let Ok(line)=line else{break};
        let response=(||->Result<Value,String>{
            let q:Value=serde_json::from_str(&line).map_err(|e|e.to_string())?;
            let op=q["op"].as_str().ok_or("op required")?;
            if op=="init"{
                let mut e=Engine::new_with_settings(q["seed"].as_i64().ok_or("seed")?,q["duration"].as_f64().ok_or("duration")?,q["map"].as_str().ok_or("map")?,
                    q.get("lineups").filter(|v|!v.is_null()).map(|v|serde_json::from_value(v.clone())).transpose().map_err(|e|e.to_string())?,
                    q.get("battle_settings").cloned())?;
                if let Some(v)=q.get("training_scenario").filter(|v|!v.is_null()){e.apply_training_scenario(v)?;}
                schedule=q["demo_events"].as_array().cloned().unwrap_or_default();cursor=0;
                while cursor<schedule.len() && schedule[cursor]["time_s"].as_f64().unwrap()<=0.{apply(&mut e,&schedule[cursor])?;cursor+=1;}
                let result=e.snapshot();engine=Some(e);return Ok(result);
            }
            let e=engine.as_mut().ok_or("engine not initialized")?;
            match op{
                "step"=>{
                    let mut applied=vec![];
                    while cursor<schedule.len() && schedule[cursor]["time_s"].as_f64().unwrap()<=e.time()+1e-8{
                        apply(e,&schedule[cursor])?;applied.push(schedule[cursor].clone());cursor+=1;
                    }
                    e.advance(&q["commands"]);
                    for ev in applied{e.events.push(json!({"type":"demo_stimulus","tick":e.tick,"time":e.time(),"stimulus":ev}));}
                    Ok(if q["static"]==false{e.dynamic_snapshot()}else{e.snapshot()})
                },
                "observation"=>{
                    let slots=q.get("slots").and_then(Value::as_array).map(|a|a.iter().filter_map(|s|s.as_u64().map(|n|n as u32)).collect::<Vec<_>>());
                    Ok(e.observation(q["team"].as_str().ok_or("team")?,slots.as_deref()))
                },
                "snapshot"=>Ok(e.snapshot()),"digest"=>Ok(json!({"digest":e.digest()})),
                "finish"=>{e.finish(q["winner"].as_str().map(str::to_string),q["reason"].as_str().unwrap_or("stopped"));Ok(e.snapshot())},
                _=>Err("unsupported demo transport op".into()),
            }
        })().unwrap_or_else(|error|json!({"error":error}));
        if serde_json::to_writer(&mut out,&response).is_err() || writeln!(&mut out).is_err() || out.flush().is_err(){break;}
    }
}
